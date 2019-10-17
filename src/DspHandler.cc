/*************************************************************************
 *
 *              Author: b51
 *                Mail: b51live@gmail.com
 *            FileName: DspHandler.cc
 *
 *          Created On: Fri Oct  4 23:06:57 2019
 *     Licensed under The MIT License [see LICENSE for details]
 *
 ************************************************************************/

#include "DspSDK/DspHandler.h"

#include <fcntl.h>
#include <stdio.h>
#include <string>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#include <errno.h>

namespace DspSDK {

DspHandler::DspHandler(const char* port_name)
    : socket_fd_(-1),
      baudrate_(DEFAULT_BAUDRATE_),
      port_name_(std::string(port_name)),
      packet_start_time_(0.0),
      packet_timeout_(0.0),
      tx_time_per_byte(0.0) {
  is_using_ = false;
}

void DspHandler::Init() {
  memset(packet_buffer_, 0, sizeof(packet_buffer_));
  memset(packet_to_proc_, 0, sizeof(packet_to_proc_));
  memset(odometers_, 0, sizeof(odometers_));

  if (!OpenPort()) {
    printf("Port init failed\n");
    return;
  }
  memset(&in_, 0, sizeof(in_));
  memset(&out_, 0, sizeof(out_));
  data_ = (unsigned short*)&in_thread_;
}

bool DspHandler::OpenPort() {
  return SetBaudRate(baudrate_);
}

void DspHandler::ClosePort() {
  if (socket_fd_ != -1)
    close(socket_fd_);
  socket_fd_ = -1;
}

void DspHandler::ClearPort() {
  tcflush(socket_fd_, TCIFLUSH);
}

bool DspHandler::SetBaudRate(const int baudrate) {
  int baud = GetCFlagBaud(baudrate);
  ClosePort();

  baudrate_ = baudrate;
  return SetupPort(baud);
}

bool DspHandler::SetupPort(int cflag_baud) {
  struct termios newtio;

  socket_fd_ = open(port_name_.c_str(), O_RDWR | O_NOCTTY);
  if (socket_fd_ < 0) {
    perror("Error opening serial port!\n");
    return false;
  }

  tcgetattr(socket_fd_, &newtio);
  tcflush(socket_fd_, TCIOFLUSH);

  // baudrate:115200
  cfsetispeed(&newtio, cflag_baud);
  cfsetospeed(&newtio, cflag_baud);

  newtio.c_cflag &= ~CSIZE;
  newtio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  newtio.c_oflag &= ~OPOST;
  newtio.c_iflag = 0;

  // 8 bits data
  newtio.c_cflag |= CS8;

  // no parity
  newtio.c_cflag &= ~PARENB;
  newtio.c_iflag &= ~INPCK;

  // 1 stop bit
  newtio.c_cflag &= ~CSTOPB;

  /////newtio.c_iflag |=INPCK;//??

  newtio.c_cc[VTIME] = 1;
  newtio.c_cc[VMIN] = 1;

  int status = tcsetattr(socket_fd_, TCSANOW, &newtio);
  if (status != 0) {
    perror("DspHandler: tcsetattr failed");
    return false;
  }
  tcflush(socket_fd_, TCIOFLUSH);
  tx_time_per_byte = (1000.0 / (double)baudrate_) * 10.0;
  return true;
}

int DspHandler::GetCFlagBaud(int baudrate) {
  switch (baudrate) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    case 460800:
      return B460800;
    case 500000:
      return B500000;
    case 576000:
      return B576000;
    case 921600:
      return B921600;
    case 1000000:
      return B1000000;
    case 1152000:
      return B1152000;
    case 1500000:
      return B1500000;
    case 2000000:
      return B2000000;
    case 2500000:
      return B2500000;
    case 3000000:
      return B3000000;
    case 3500000:
      return B3500000;
    case 4000000:
      return B4000000;
    default:
      return -1;
  }
}

double DspHandler::GetCurrentTime() {
  struct timespec tv;
  clock_gettime(CLOCK_REALTIME, &tv);
  return ((double)tv.tv_sec * 1000.0 + (double)tv.tv_nsec * 0.001 * 0.001);
}

double DspHandler::GetTimeSinceStart() {
  double time;

  time = GetCurrentTime() - packet_start_time_;
  if (time < 0.0) packet_start_time_ = GetCurrentTime();

  return time;
}

void DspHandler::DspThread() {
  memcpy(&in_thread_, &in_, sizeof(in_));
  tcflush(socket_fd_, TCIFLUSH);

  start_packet_.id = ID_DSP;
  start_packet_.instruction = INST_STATE_SWAP;
  start_packet_.length = (sizeof(struct StateSwapInput) + 2);
  for (size_t i = 0; i < sizeof(struct StateSwapInput) / sizeof(unsigned short);
       i++) {
    start_packet_.parameter[2 * i] = (data_[i] & 0xff);
    start_packet_.parameter[2 * i + 1] = ((data_[i] >> 8) & 0xff);
  }
  retval_ = CompressPacket(info_, &start_packet_);
  retval_ = WritePort(info_, retval_);

  struct timeval tv;
  fd_set rdfs;

  tv.tv_sec = 0;
  tv.tv_usec = 200000;
  process_sucess_ = false;
  do {
    FD_ZERO(&rdfs);
    FD_SET(socket_fd_, &rdfs);

    retval_ = select(socket_fd_ + 1, &rdfs, nullptr, nullptr, &tv);

    if (retval_ < 0) {
      tv.tv_sec = 0;
      tv.tv_usec = 200000;
      continue;
    } else if (retval_ == 0) {
      printf("%d waiting reply timeout warning ******\n", retval_);
    }
    retval_ = ReadPort(info_, sizeof(info_));
    if (retval_ <= 0) {
      perror("Receive string failed from serials");
      continue;
    }

    for (int i = 0; i < retval_; i++) {
      if (PacketReconstrct(info_[i])) {
        DSP_STATUS_PACKET* recv = nullptr;
        if (PacketInterprete(&recv)) {
          process_sucess_ = PostProcessingStateSwap(recv);
        }
      }
    }  // end of for loop
  } while (!process_sucess_);

  if (!process_sucess_) PostProcessingTimeout();
}

size_t DspHandler::CompressPacket(unsigned char* output, DSP_INST_PACKET* packet) {
  unsigned char check_sum = 0;
  unsigned char* packet_char = (unsigned char*)packet;
  size_t ret = 0;

  check_sum += packet->id + packet->instruction + packet->length;

  for (int i = 0; i < packet->length - 2; i++) {
    check_sum += packet->parameter[i];
  }
  check_sum = ~check_sum;
  ret = packet->length + 4;
  output[0] = 0xff;
  output[1] = 0xff;
  for (int i = 0; i < packet->length + 1; i++) {
    output[i + 2] = packet_char[i];
  }
  output[packet->length + 3] = check_sum;
  return ret;
}

bool DspHandler::PacketInterprete(DSP_STATUS_PACKET** container) {
  DSP_STATUS_PACKET* raw_packet = (DSP_STATUS_PACKET*)(packet_to_proc_ + 2);
  unsigned char check_sum = raw_packet->parameter[raw_packet->length - 2];
  unsigned char check_sum_cal = 0;
  for (int i = 0; i < raw_packet->length + 1; i++) {
    check_sum_cal += packet_to_proc_[2 + i];
  }
  check_sum_cal = ~check_sum_cal;

  if (check_sum_cal == check_sum) {
    *container = raw_packet;
    return true;
  }
  return false;
}

bool DspHandler::PacketReconstrct(unsigned char received) {
  if (number_received_ >= 5) {
    DSP_STATUS_PACKET* info;
    info = (DSP_STATUS_PACKET*)(packet_buffer_ + 2);
    if (number_received_ == (size_t)info->length + 4 - 1) {
      packet_buffer_[number_received_++] = received;
      memcpy(packet_to_proc_, packet_buffer_, number_received_);
      number_received_ = 0;
      return true;
    } else if (number_received_ < (size_t)info->length + 4 - 1) {
      packet_buffer_[number_received_] = received;
    }
  } else if (number_received_ >= 3) {
    packet_buffer_[number_received_++] = received;
  } else if (number_received_ >= 2) {
    if (received == ID_DSP) {
      packet_buffer_[number_received_] = received;
    } else {
      number_received_ = 0;
      return false;
    }
  } else {
    if (received == 0xff) {
      packet_buffer_[number_received_] = received;
    } else {
      number_received_ = 0;
      return false;
    }
  }
  return false;
}

bool DspHandler::PostProcessingStateSwap(DSP_STATUS_PACKET* packet) {
  unsigned short* data = nullptr;
  if (packet->infomation & PACKET_TYPE_MASK) {
    switch (packet->infomation & ~PACKET_TYPE_MASK) {
      case INST_STATE_SWAP: {
        data = (unsigned short*)&out_;
        for (size_t i = 0;
             i < sizeof(struct StateSwapOutput) / sizeof(unsigned short); i++) {
          data[i] =
              packet->parameter[2 * i] + (packet->parameter[2 * i + 1] << 8);
        }
      } break;  // end of case INST_STATE_SWAP
      default:
        break;
    }  // end of switch
  }
  return true;
}

bool DspHandler::ClearOdometers() {
  for (int i = 0; i < MAX_ODOMETER_RECORD; i++)
    if (odometers_[i].flag & ODOMETER_RECORD_VALID)
      memset(&odometers_[i].lastRecord, 0, sizeof(struct GaitEffect));

  return true;
}

bool DspHandler::PostProcessingTimeout() {
  out_.stsReg1 = in.ctrReg1;
  out_.stsReg2 = in.ctrReg2;
  memcpy(&out_.spcSts, &in.spcInst, sizeof(struct SpecialGaitCommand));
  memcpy(&out_.headSts, &in.headInst, sizeof(struct SpecialGaitCommand));
  if (out.stsReg1 & COPY_RESET_ODOMETER)
    ClearOdometers();
  return true;
}

Eigen::Vector2d DspHandler::GetHeadPos() {
  Eigen::Vector2d head_pos;
  head_pos[0] = out.headSts.pitch / 512.;
  head_pos[1] = out.headSts.yaw / 512.;
  return head_pos;
}

Eigen::Vector3d DspHandler::GetOdometry() {
  Eigen::Vector3d odometry;
  odometry[0] = out.odometer.xOffset / 1000.;
  odometry[1] = out.odometer.yOffset / 1000.;
  odometry[2] = out.odometer.thetaOffset / 512.;
  return odometry;
}

Eigen::Vector3d DspHandler::GetVelocity() {
  Eigen::Vector3d velocity;
  velocity[0] = out.dirSts.xOffset / 1000.;
  velocity[1] = out.dirSts.yOffset / 1000.;
  velocity[2] = out.dirSts.thetaOffset / 512.;
  return velocity;
}

Eigen::Vector3d DspHandler::GetImuAngle() {
  Eigen::Vector3d imu_angle;
  imu_angle[0] = out.sensors.incline[0] / 512.;
  imu_angle[0] = out.sensors.incline[1] / 512.;
  imu_angle[0] = out.sensors.incline[2] / 512.;
  return imu_angle;
}

Eigen::Matrix<double, 7, 1> DspHandler::GetBodyPose() {
  Eigen::Matrix<double, 7, 1> body_pose;
  body_pose[0] = out.isLeft;
  body_pose[1] = out.torsoPose.offset.x / 1000.;
  body_pose[2] = out.torsoPose.offset.y / 1000.;
  body_pose[3] = out.torsoPose.offset.z / 1000.;
  body_pose[4] = out.torsoPose.pose.alpha / 512.;
  body_pose[5] = out.torsoPose.pose.beta / 512.;
  body_pose[6] = out.torsoPose.pose.theta / 512.;
  return body_pose;
}

std::vector<bool> DspHandler::GetWalkKick() {
  std::vector<bool> walk_kick;
  walk_kick.emplace_back(out_.stsReg1 & WALK_KICK_LEFT);
  walk_kick.emplace_back(out_.stsReg1 & WALK_KICK_RIGHT);
  return walk_kick;
}

std::vector<bool> DspHandler::GetSpecialGaitState() {
  std::vector<bool> state;
  state.emplace_back(out_.stsReg2 & SPECIAL_GAIT_PENDING);
  state.emplace_back(out_.stsReg2 & GAIT_RESET_PENDING);
  state.emplace_back(out_.stsReg2 & RESET_ODOMETER_PENDING);
  state.emplace_back(out_.stsReg2 & WALK_KICK_PENDING);
  return state;
}

void DspHandler::SetVelocity(const std::vector<double>& velocity) {
  in_.dirInst.xOffset = short(velocity[0] * 1000.);
  in_.dirInst.yOffset = short(velocity[1] * 1000.);
  in_.dirInst.thetaOffset = short(velocity[2] * 512.);
}

void DspHandler::SetVelocity(double x, double y, double yaw) {
  in_.dirInst.xOffset = short(x * 1000.);
  in_.dirInst.yOffset = short(y * 1000.);
  in_.dirInst.thetaOffset = short(yaw * 512.);
}

void DspHandler::SetHeadPos(const std::vector<double>& pos) {
  in_.headInst.pitch = short(pos[0] * 512.);
  in_.headInst.yaw = short(pos[1] * 512.);
}

void DspHandler::SetHeadPos(double pitch, double yaw) {
  in_.headInst.pitch = short(pitch * 512.);
  in_.headInst.yaw = short(yaw * 512.);
}

void DspHandler::SetSpecialGaitId(const std::vector<int>& special_gait) {
  in_.spcInst.id = short(special_gait[0]);
  in_.spcInst.times = short(special_gait[1]);
}

void DspHandler::SetSpecialGaitId(int id, int repeat) {
  in_.spcInst.id = short(id);
  in_.spcInst.times = short(repeat);
}

} //  namespace DspSDK
