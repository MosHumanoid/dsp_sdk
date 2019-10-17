/*************************************************************************
 *
 *              Author: b51
 *                Mail: b51live@gmail.com
 *            FileName: DspHandler.h
 *
 *          Created On: Fri Oct  4 22:58:23 2019
 *     Licensed under The MIT License [see LICENSE for details]
 *
 ************************************************************************/

#ifndef DSP_HANDLER_H_
#define DSP_HANDLER_H_

#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <string>
#include <vector>
#include <Eigen/Core>

#include "DspCmd.h"

namespace DspSDK {

class DspHandler {
 public:
  static const int DEFAULT_BAUDRATE_ = 115200;
  static const int LATENCY_TIMER = 16;  // msec (USB latency timer)

  static DspHandler* GetDspHandler(const char* port_name);

  DspHandler(const char* port_name);
  ~DspHandler() { ClosePort(); }

  void Init();

  bool OpenPort();

  void ClosePort();

  void ClearPort();

  void SetPortName(const char* port_name);

  inline std::string GetPortName() const { return port_name_; }

  bool SetBaudRate(const int baudrate);

  inline int GetBaudRate() const { return baudrate_; }

  inline int ReadPort(uint8_t* packet, int length) {
    return read(socket_fd_, packet, length);
  };

  inline int WritePort(uint8_t* packet, int length) {
    return write(socket_fd_, packet, length);
  };

  void SetPacketTimeout(uint16_t packet_length) {
    packet_start_time_ = GetCurrentTime();
    packet_timeout_ = (tx_time_per_byte * (double)packet_length) +
                      (LATENCY_TIMER * 2.0) + 2.0;
  }

  void SetPacketTimeout(double msec) {
    packet_start_time_ = GetCurrentTime();
    packet_timeout_ = msec;
  }

  bool IsPacketTimeout() {
    if (GetTimeSinceStart() > packet_timeout_) {
      packet_timeout_ = 0;
      return true;
    }
    return false;
  };

  void DspThread();

  size_t CompressPacket(unsigned char* output, DSP_INST_PACKET* packet);

  bool PacketInterprete(DSP_STATUS_PACKET** container);

  bool PacketReconstrct(unsigned char received);

  bool PostProcessingStateSwap(DSP_STATUS_PACKET* packet);

  bool PostProcessingTimeout();

  bool ClearOdometers();

  bool is_using_;

  /**
   *  GetHeadPos
   *  return vector[0]: pitch in rad
   *               [1]: yaw in rad
   */
  Eigen::Vector2d GetHeadPos();

  /**
   *  GetOdometry
   *  return vector[0]: x odometry in m
   *               [1]: y odometry in m
   *               [2]: theta odometry in rad
   */
  Eigen::Vector3d GetOdometry();

  /**
   *  GetVelocity
   *  return vector[0]: x velocity in m/s
   *               [1]: y velocity in m/s
   *               [2]: theta velocity in rad/s
   */
  Eigen::Vector3d GetVelocity();

  /**
   *  GetVelocity
   *  return vector[0]: roll in rad
   *               [1]: pitch in rad (pitch > 0, robot face to ground)
   *               [2]: yaw in rad
   */
  Eigen::Vector3d GetImuAngle();

  /**
   *  GetBodyPose
   *  return vector[0]: is left foot ahead?
   *               [1]: x rigid body pose in m
   *               [2]: y rigid body pose in m
   *               [3]: z rigid body pose in m
   *               [4]: roll in rad
   *               [5]: pitch in rad
   *               [6]: yaw in rad
   */
  Eigen::Matrix<double, 7, 1 > GetBodyPose();

  /**
   *  GetGaitDirectionValid
   *  return is gait direction valid
   */
  inline bool GetGaitDirectionValid() {
    return (out_.stsReg1 & GAIT_DIRECTION_VALID);
  }

  /**
   *  GetSpecialGaitValid
   *  return is special gait valid
   */
  inline bool GetSpecialGaitValid() {
    return (out_.stsReg1 & SPECIAL_GAIT_VALID);
  }

  /**
   *  GetGaitResetValid
   *  return is gait reset valid
   */
  inline bool GetGaitResetValid() {
    return (out_.stsReg1 & GAIT_RESET_VALID);
  }

  /**
   *  GetOdometryReset
   *  return is odometry reset
   */
  inline bool GetOdometryReset() {
    return (out_.stsReg1& COPY_RESET_ODOMETER);
  }

  /**
   *  GetHeadMoveValid
   *  return is head move valid
   */
  inline bool GetHeadMoveValid() {
    return (out_.stsReg1 & HEAD_MOVE_VALID);
  }

  /**
   *  GetTorqueEnabelValid
   *  return is torque enable valid
   */
  inline bool GetTorqueEnabelValid() {
    return (out_.stsReg1 & TORQUE_ENABLE_VALID);
  }

  /**
   *  GetSensorEnableValid
   *  return is IMU enabled
   */
  inline bool GetSensorEnableValid() {
    return (out_.stsReg1 & SENSOR_ENABLE_VALID);
  }

  /**
   *  GetWalkKick
   *  return vector[0]: is walk kick with left foot
   *               [1]: is walk kick with right foot
   */
  std::vector<bool> GetWalkKick();

  /**
   *  GetSpecialGaitState
   *  return vector[0]: is special gait pending
   *               [1]: is gait reset pending
   *               [2]: is odometry reset pending
   *               [3]: is walk kick pending
   */
  std::vector<bool> GetSpecialGaitState();

  /**
   *  SetVelocity
   *  input velocity[0]: x speed in m/s
   *                [1]: y speed in m/s
   *                [2]: yaw speed in rad/s
   */
  void SetVelocity(const std::vector<double>& velocity);
  void SetVelocity(double x, double y, double yaw);

  /**
   *  SetHeadPose
   *  input pose[0]: head pitch in rad
   *            [1]: head yaw in rad
   */
  void SetHeadPos(const std::vector<double>& pos);
  void SetHeadPos(double pitch, double yaw);

  /**
   *  SetSpecialGaitId
   *  input special_gait[0]: special gait id
   *                    [1]: how many times will repeats
   */
  void SetSpecialGaitId(const std::vector<int>& special_gait);
  void SetSpecialGaitId(int id, int repeat);

  /**
   *  Set some states to dsp
   */
  inline void SetHeadMoveValid(bool valid) {
    if (valid)
      in_.ctrReg1 |= HEAD_MOVE_VALID;
    else
      in_.ctrReg1 &= ~HEAD_MOVE_VALID;
  }
  inline void SetSensorEnableValid(bool valid) {
    if (valid)
      in_.ctrReg1 |= SENSOR_ENABLE_VALID;
    else
      in_.ctrReg1 &= ~SENSOR_ENABLE_VALID;
  }
  inline void ResetOdometry(bool valid) {
    if (valid)
      in_.ctrReg1 |= COPY_RESET_ODOMETER;
    else
      in_.ctrReg1 &= ~COPY_RESET_ODOMETER;
  }
  inline void SetSpecialGaitValid(bool valid) {
    if (valid)
      in_.ctrReg1 |= SPECIAL_GAIT_VALID;
    else
      in_.ctrReg1 &= ~SPECIAL_GAIT_VALID;
  }
  inline void SetWalkKickLeft(bool valid) {
    if (valid)
      in_.ctrReg1 |= WALK_KICK_LEFT;
    else
      in_.ctrReg1 &= ~WALK_KICK_LEFT;
  }
  inline void SetWalkKickRight(bool valid) {
    if (valid)
      in_.ctrReg1 |= WALK_KICK_RIGHT;
    else
      in_.ctrReg1 &= ~WALK_KICK_RIGHT;
  }
  inline void SetTorqueEnable(bool valid) {
    if (valid)
      in_.ctrReg1 |= TORQUE_ENABLE_VALID;
    else
      in_.ctrReg1 &= ~TORQUE_ENABLE_VALID;
  }
  inline void SetGaitValid(bool valid) {
    if (valid)
      in_.ctrReg1 |= GAIT_DIRECTION_VALID;
    else
      in_.ctrReg1 &= ~GAIT_DIRECTION_VALID;
  }
  inline void ResetGait(bool valid) {
    if (valid)
      in_.ctrReg1 |= GAIT_RESET_VALID;
    else
      in_.ctrReg1 &= ~GAIT_RESET_VALID;
  }

 private:
  bool SetupPort(const int cflag_baud);
  int GetCFlagBaud(const int baudrate);

  double GetCurrentTime();
  double GetTimeSinceStart();

  int socket_fd_;
  int baudrate_;
  std::string port_name_;

  double packet_start_time_;
  double packet_timeout_;
  double tx_time_per_byte;

  struct StateSwapInput in_thread_, in_;
  struct StateSwapOutput out_;
  struct OdometerRecord odometers_[MAX_ODOMETER_RECORD];
  DSP_INST_PACKET start_packet_;
  unsigned int number_received_;
  unsigned char info_[sizeof(DSP_INST_PACKET) + 2];
  bool process_sucess_;
  unsigned short* data_;
  int retval_;

  /**
   *  packet_buffer_: Reconstructed bytes info to packet from dsp
   *  packet_to_proc_: Recontructed packet to process
   */
  char packet_buffer_[sizeof(DSP_INST_PACKET) + 2];
  char packet_to_proc_[sizeof(DSP_INST_PACKET) + 2];
};

} //  namespace DspSDK

#endif
