#include <lua.hpp>

#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>
#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/utility/binary.hpp>
#include <string>
#include <vector>
#include "DspCode.h"
#include "DspControl.h"
#include "RobotParameter.h"
#include "linuxdef.h"

using namespace boost::interprocess;

typedef double value_t;

//----------------------------------------------------------------------
// CompressPacket 打包
//
static UINT uiNumReceived = 0;
static DSP_INST_PACKET startPacket;
static BYTE info[sizeof(DSP_INST_PACKET) + 2];
static struct StateSwapInput in_thread;
static PUSHORT pdata = NULL;
static BOOL processSuccessful = FALSE;
static struct timeval tv;
static fd_set rdfs;
static int sio_fd = -1;
static int retval;
static int fd = -1;

int init_rs232(void) {
  int status;
  struct termios opt;
  if (sio_fd != -1) return sio_fd;
  sio_fd = open(RS232_DEVICE_NAME, O_RDWR | O_NOCTTY);
  // printf("sio_fd=%d\n",sio_fd);
  if (sio_fd == -1) {
    perror("serial port open failed");
    return -1;
  }

  tcgetattr(sio_fd, &opt);
  tcflush(sio_fd, TCIOFLUSH);

  // baudrate:115200
  cfsetispeed(&opt, B115200);
  cfsetospeed(&opt, B115200);

  opt.c_cflag &= ~CSIZE;
  opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  opt.c_oflag &= ~OPOST;
  opt.c_iflag = 0;

  // 8 bits data
  opt.c_cflag |= CS8;

  // no parity
  opt.c_cflag &= ~PARENB;
  opt.c_iflag &= ~INPCK;

  // 1 stop bit
  opt.c_cflag &= ~CSTOPB;

  /////opt.c_iflag |=INPCK;//??

  opt.c_cc[VTIME] = 1;
  opt.c_cc[VMIN] = 1;

  status = tcsetattr(sio_fd, TCSANOW, &opt);
  if (status != 0) {
    perror("tcsetattr failed");
    return -1;
  }
  tcflush(sio_fd, TCIOFLUSH);
  return sio_fd;
}

int destroy_rs232(void) {
  if (sio_fd != -1) close(sio_fd);
  sio_fd = -1;
  return 0;
}

UINT CompressPacket(PBYTE output, DSP_INST_PACKET* packet) {
  BYTE checkSum = 0;
  PBYTE pPacketChar = (PBYTE)packet;
  UINT ret = 0;
  int i = 0;

  checkSum += packet->id + packet->instruction + packet->length;
  for (i = 0; i < packet->length - 2; i++) checkSum += packet->parameter[i];
  checkSum = ~checkSum;

  ret = packet->length + 4;
  // printf("packet->length=%d\n",packet->length);
  // printf("ret=%d\n",ret);
  output[0] = 0xff;
  output[1] = 0xff;

  for (i = 0; i < packet->length + 1; i++) output[i + 2] = pPacketChar[i];
  output[packet->length + 3] = checkSum;
  return ret;
}

BOOL PacketInterprete(DSP_STATUS_PACKET** container) {
  DSP_STATUS_PACKET* pRawPacket = (DSP_STATUS_PACKET*)(packetToProc + 2);
  BYTE checkSum = pRawPacket->parameter[pRawPacket->length - 2];
  BYTE checkSumCal = 0;
  int i = 0;

  for (i = 0; i < pRawPacket->length + 1; i++)
    checkSumCal += packetToProc[2 + i];
  checkSumCal = ~checkSumCal;

  if (checkSumCal == checkSum) {
    *container = pRawPacket;
    return TRUE;
  }
  return FALSE;
}

BOOL PacketReconstrct(BYTE bReceived) {
  // need change
  // UINT uiNumReceived=0;
  // printf("%d ",bReceived);

  if (uiNumReceived >= 5) {
    DSP_STATUS_PACKET* info;
    info = (DSP_STATUS_PACKET*)(packetBuffer + 2);
    if (uiNumReceived == (UINT)info->length + 4 - 1) {
      packetBuffer[uiNumReceived++] = bReceived;
      memcpy(packetToProc, packetBuffer, uiNumReceived);
      uiNumReceived = 0;
      return TRUE;
    } else if (uiNumReceived < (UINT)info->length + 4 - 1) {
      packetBuffer[uiNumReceived++] = bReceived;
    }
  } else if (uiNumReceived >= 3) {
    packetBuffer[uiNumReceived++] = bReceived;
  } else if (uiNumReceived >= 2) {
    if (bReceived == ID_DSP) {
      packetBuffer[uiNumReceived++] = bReceived;
    } else {
      uiNumReceived = 0;
      return FALSE;
    }
  } else {
    if (bReceived == 0xff) {
      packetBuffer[uiNumReceived++] = bReceived;
    } else {
      uiNumReceived = 0;
      return FALSE;
    }
  }
  return FALSE;
}

BOOL DiffRotMatrix(struct GaitEffect* pm1, struct GaitEffect* pm2,
                   struct GaitEffect* pm3) {
  float rad = 0;
  float deltax = 0;
  float deltay = 0;
  struct GaitEffect temp;
  memset(&temp, 0, sizeof(temp));
  // get pm1 relative to pm2 and restore it in pm3
  temp.thetaOffset = pm1->thetaOffset - pm2->thetaOffset;
  rad = (float)pm2->thetaOffset / 512.0;
  deltax = (float)pm1->xOffset - (float)pm2->xOffset;
  deltay = (float)pm1->yOffset - (float)pm2->yOffset;
  temp.xOffset = (SHORT)(deltax * cos(rad) + deltay * sin(rad));
  temp.yOffset = (SHORT)(-deltax * sin(rad) + deltay * cos(rad));
  memcpy(pm3, &temp, sizeof(temp));
  return TRUE;
}

BOOL ResetOdometers(struct GaitEffect* plastEffect) {
  int i = 0;
  for (i = 0; i < MAX_ODOMETER_RECORD; i++) {
    if (odometers[i].flag & ODOMETER_RECORD_VALID)
      DiffRotMatrix(&odometers[i].lastRecord, plastEffect,
                    &odometers[i].lastRecord);
  }

  return TRUE;
}

BOOL ClearOdometers() {
  int i = 0;
  for (i = 0; i < MAX_ODOMETER_RECORD; i++)
    if (odometers[i].flag & ODOMETER_RECORD_VALID)
      memset(&odometers[i].lastRecord, 0, sizeof(struct GaitEffect));

  return TRUE;
}

BOOL PostProcessingTimeout(void) {
  out.stsReg1 = in.ctrReg1;
  out.stsReg2 = in.ctrReg2;
  memcpy(&out.spcSts, &in.spcInst, sizeof(struct SpecialGaitCommand));
  memcpy(&out.headSts, &in.headInst, sizeof(struct SpecialGaitCommand));
  if (out.stsReg1 & COPY_RESET_ODOMETER) ClearOdometers();
  return TRUE;
}

BOOL PostProcessingStateSwap(DSP_STATUS_PACKET* pkt) {
  PUSHORT pdata = NULL;
  unsigned int i = 0;
  if (pkt->infomation & PACKET_TYPE_MASK) {
    switch (pkt->infomation & ~PACKET_TYPE_MASK) {
      case INST_STATE_SWAP: {
        // reply is received
        pdata = (PUSHORT)&out;
        for (i = 0; i < sizeof(struct StateSwapOutput) / sizeof(USHORT); i++) {
          pdata[i] = (pkt->parameter[2 * i] + (pkt->parameter[2 * i + 1] << 8));
          // printf("%d ",pdata[i]);
        }
        //			printf("\n");
        // printf("out.velocity.x=%d,out.velocity.y=%d,out.veloctiy.theat=%d\n",out.dirSts.xOffset,out.dirSts.yOffset,out.dirSts.thetaOffset);
        /*
                                //other processing of the receiveing packet
                                if(out.stsReg1&COPY_RESET_ODOMETER)
                                        ResetOdometers(&out.odometer);


                                //post handling of inst packet
                                //if special gait command is sent or odometer is
           reset
                                //no further command or reset is executed
                                in.ctrReg1&=~(out.stsReg1&SPECIAL_GAIT_VALID);
                                in.ctrReg1&=~(out.stsReg1&GAIT_DIRECTION_VALID);
                                in.ctrReg1&=~(out.stsReg1&COPY_RESET_ODOMETER);
        */
      } break;
      default:
        break;
    }
  }
  return TRUE;
}

void EnterPreparingState(void) {
  memset(&in, 0, sizeof(in));
  memset(&out, 0, sizeof(out));
  return;
}

void PreProcessingStateSwap(struct StateSwapInput* pin) {
  if (pGetNextFrame != NULL)
    (*pGetNextFrame)(&in.headInst.pitch, &in.headInst.yaw);
  // other processing of the sending packet
  /*
          if(
                  out.odometer.xOffset>10000||
                  out.odometer.xOffset<-10000||
                  out.odometer.yOffset>10000||
                  out.odometer.yOffset<-10000||
                  out.odometer.thetaOffset>10000||
                  out.odometer.thetaOffset<-10000
                  )
                  in.ctrReg1|=COPY_RESET_ODOMETER;
  */
  /*
          if(in.ctrReg1&GAIT_DIRECTION_VALID)
                  printf("gait direction command is released\n");
          if(in.ctrReg1&SPECIAL_GAIT_VALID)
                  printf("special gait command is released\n");
  */
  memcpy(pin, &in, sizeof(in));
  return;
}
// dcmEntry()

void BeginDspThread() {
  memset(packetBuffer, 0, sizeof(packetBuffer));
  memset(packetToProc, 0, sizeof(packetToProc));
  memset(odometers, 0, sizeof(odometers));
  pGetNextFrame = NULL;

  if (-1 == (fd = init_rs232()))  // dcm对应的设定波特率什么的
  {
    printf("rs232 port init failed\n");
    return;
  }
  // printf("fd=%d\n",fd);
  EnterPreparingState();  // in.out分别是抓包和收包的
  pdata = (PUSHORT)&in_thread;
}

// dcmupdate()
void DspThread() {
  // previous processing of state swaping packet
  PreProcessingStateSwap(&in_thread);
  // printf("fd=%d\n",fd);
  // clear all data received but not read
  tcflush(fd, TCIFLUSH);

  startPacket.id = ID_DSP;
  startPacket.instruction = INST_STATE_SWAP;
  startPacket.length = (sizeof(struct StateSwapInput) + 2);
  for (unsigned int i = 0; i < sizeof(struct StateSwapInput) / sizeof(USHORT);
       i++) {
    startPacket.parameter[2 * i] = (pdata[i] & 0xff);
    startPacket.parameter[2 * i + 1] = ((pdata[i] >> 8) & 0xff);
    // printf("%d ",pdata[i]);
  }
  // printf("\n");

  retval = CompressPacket(info, &startPacket);
  // printf("compass retval=%d fd=%d\n",retval,fd);

  retval = write(fd, info, retval);
  // printf("write retval=%d\n",retval);
  //		printf("writedown\n");
  tv.tv_sec = 0;
  tv.tv_usec = 200000;
  processSuccessful = FALSE;

  do {
    FD_ZERO(&rdfs);
    FD_SET(fd, &rdfs);

    retval = select(fd + 1, &rdfs, NULL, NULL, &tv);
    // printf("select retval=%d\n",retval);
    if (retval < 0) {
      tv.tv_sec = 0;
      tv.tv_usec = 200000;
      continue;
    }

    else if (retval == 0) {
      // waiting for reply timeout happened
      printf("%dwaiting reply timeout warning ********\n", retval);
      break;
    }
    retval = read(fd, info, sizeof(info));
    // printf("read retval=%d ++++++++++++++++++++++++++++++\n",retval);
    if (retval <= 0) {
      // continue to wait for valid character
      perror("receive string failed from serials");
      continue;
    }
    // printf("%d bytes received\n",retval);
    // recv message is read, processing is following
    for (int i = 0; i < retval; i++) {
      if (PacketReconstrct(info[i])) {
        DSP_STATUS_PACKET* recv = NULL;
        if (PacketInterprete(&recv)) {
          processSuccessful = PostProcessingStateSwap(recv);
          // printf("state swap packet receivecd\n");
        }
      }
    }
    //			printf("\n");
  } while (!processSuccessful);
  // failure of receiving proper packet
  //                printf("receive success\n");
  if (!processSuccessful) PostProcessingTimeout();

  // usleep(50000);
}

static int lua_pass_state(lua_State* L) {
  luaL_checktype(L, 1, LUA_TTABLE);
  int nIndex = lua_gettop(L);
  lua_pushnil(L);
  int i = 0;
  while (0 != lua_next(L, nIndex)) {
    // structCArray *a=(structCArray*)lua_touserdata(L,-1);
    stateIn[i] = (structCArray*)lua_touserdata(L, -1);
    stateName[i] = (char*)lua_tostring(L, -2);
    //        printf("%s:%d:%c
    //        ",stateName[i],int(stateIn[i]->ptr[0]),stateIn[i]->type);
    //        printf("\n");
    lua_pop(L, 1);
    i++;
    stateNum = i;
  }
  setStateToIn();
  //  printf("statenum=%d\n",stateNum);
  return 1;
}

static int lua_pass_para(lua_State* L) {
  luaL_checktype(L, 1, LUA_TTABLE);
  int nIndex = lua_gettop(L);
  lua_pushnil(L);
  int i = 0;
  while (0 != lua_next(L, nIndex)) {
    // structCArray *a=(structCArray*)lua_touserdata(L,-1);
    paraIn[i] = (structCArray*)lua_touserdata(L, -1);
    paraName[i] = (char*)lua_tostring(L, -2);
    //        printf("%s:%d:%c
    //        ",paraName[i],int(paraIn[i]->ptr[0]),paraIn[i]->type);
    //        printf("\n");
    lua_pop(L, 1);
    i++;
    paraNum = i;
  }
  setParaToIn();
  //  printf("paranum=%d\n",paraNum);
  return 1;
}

void setStateToIn() {
  int i = 0;
  for (i = 0; i < stateNum; i++) {
    if (!strcmp("headValid", stateName[i])) {
      if (int(stateIn[i]->ptr[0]) == 1) {
        in.ctrReg1 |= HEAD_MOVE_VALID;
      } else {
        in.ctrReg1 &= ~HEAD_MOVE_VALID;
      }
    } else if (!strcmp("sensorEnable", stateName[i])) {
      if (int(stateIn[i]->ptr[0]) == 1) {
        in.ctrReg1 |= SENSOR_ENABLE_VALID;
      } else {
        in.ctrReg1 &= ~SENSOR_ENABLE_VALID;
      }
    } else if (!strcmp("odometerReset", stateName[i])) {
      if (int(stateIn[i]->ptr[0]) == 1) {
        in.ctrReg1 |= COPY_RESET_ODOMETER;
      } else {
        in.ctrReg1 &= ~COPY_RESET_ODOMETER;
      }
    } else if (!strcmp("specialValid", stateName[i])) {
      if (int(stateIn[i]->ptr[0]) == 1) {
        in.ctrReg1 |= SPECIAL_GAIT_VALID;
      } else {
        in.ctrReg1 &= ~SPECIAL_GAIT_VALID;
      }
    } else if (!strcmp("walkkick", stateName[i])) {
      if (int(stateIn[i]->ptr[0]) == 1) {
        in.ctrReg1 |= WALK_KICK_LEFT;
      } else {
        in.ctrReg1 &= ~WALK_KICK_LEFT;
      }
      if (int(stateIn[i]->ptr[1]) == 1) {
        in.ctrReg1 |= WALK_KICK_RIGHT;
      } else {
        in.ctrReg1 &= ~WALK_KICK_RIGHT;
      }
    } else if (!strcmp("torqueEnable", stateName[i])) {
      if (int(stateIn[i]->ptr[0]) == 1) {
        in.ctrReg1 |= TORQUE_ENABLE_VALID;
      } else {
        in.ctrReg1 &= ~TORQUE_ENABLE_VALID;
      }
    } else if (!strcmp("gaitValid", stateName[i])) {
      if (int(stateIn[i]->ptr[0]) == 1) {
        in.ctrReg1 |= GAIT_DIRECTION_VALID;
      } else {
        in.ctrReg1 &= ~GAIT_DIRECTION_VALID;
      }
    } else if (!strcmp("gaitReset", stateName[i])) {
      if (int(stateIn[i]->ptr[0]) == 1) {
        in.ctrReg1 |= GAIT_RESET_VALID;
      } else {
        in.ctrReg1 &= ~GAIT_RESET_VALID;
      }
    } else
      continue;
  }
  //  printf("in.ctrReg1=%d\n",in.ctrReg1);
}
void setParaToIn() {
  int i = 0;
  for (i = 0; i < paraNum; i++) {
    if (!strcmp("velocity", paraName[i])) {
      in.dirInst.xOffset = SHORT(paraIn[i]->ptr[0]);
      in.dirInst.yOffset = SHORT(paraIn[i]->ptr[1]);
      in.dirInst.thetaOffset = SHORT(paraIn[i]->ptr[2]);
      // printf("in.dirInst.XOffset=%d in.dirInst.yOffset=%d
      // in.dirInst.thetaOffset=%d\n",in.dirInst.xOffset,
      // in.dirInst.yOffset,in.dirInst.thetaOffset);
    } else if (!strcmp("headpos", paraName[i])) {
      in.headInst.pitch = SHORT(paraIn[i]->ptr[0]);
      in.headInst.yaw = SHORT(paraIn[i]->ptr[1]);
      // printf("in.headInst.pitch=%d
      // in.headInst.yaw=%d\n",in.headInst.pitch,in.headInst.yaw);
    } else if (!strcmp("gaitID", paraName[i])) {
      in.spcInst.id = SHORT(paraIn[i]->ptr[0]);
      in.spcInst.times = SHORT(paraIn[i]->ptr[1]);
      // printf("in.spcInst.id=%d
      // in.spcInst.times=%d\n",in.spcInst.id,in.spcInst.times);
    }
  }
  // printf("end\n");
}

static int lua_get_gdv_from_c(lua_State* L) {
  if (out.stsReg1 & GAIT_DIRECTION_VALID)
    lua_pushnumber(L, 1);
  else
    lua_pushnumber(L, 0);
  return 1;
}
static int lua_get_sgv_from_c(lua_State* L) {
  if (out.stsReg1 & SPECIAL_GAIT_VALID)
    lua_pushnumber(L, 1);
  else
    lua_pushnumber(L, 0);
  return 1;
}
static int lua_get_grv_from_c(lua_State* L) {
  if (out.stsReg1 & GAIT_RESET_VALID)
    lua_pushnumber(L, 1);
  else
    lua_pushnumber(L, 0);
  return 1;
}
static int lua_get_cro_from_c(lua_State* L) {
  if (out.stsReg1 & COPY_RESET_ODOMETER)
    lua_pushnumber(L, 1);
  else
    lua_pushnumber(L, 0);
  return 1;
}
static int lua_get_hmv_from_c(lua_State* L) {
  if (out.stsReg1 & HEAD_MOVE_VALID)
    lua_pushnumber(L, 1);
  else
    lua_pushnumber(L, 0);
  return 1;
}
static int lua_get_tev_from_c(lua_State* L) {
  if (out.stsReg1 & TORQUE_ENABLE_VALID)
    lua_pushnumber(L, 1);
  else
    lua_pushnumber(L, 0);
  return 1;
}
static int lua_get_sev_from_c(lua_State* L) {
  if (out.stsReg1 & SENSOR_ENABLE_VALID)
    lua_pushnumber(L, 1);
  else
    lua_pushnumber(L, 0);
  return 1;
}
static int lua_get_walk_kick(lua_State* L) {
  if (out.stsReg1 & WALK_KICK_LEFT)
    lua_pushnumber(L, 1);
  else
    lua_pushnumber(L, 0);
  if (out.stsReg1 & WALK_KICK_RIGHT)
    lua_pushnumber(L, 1);
  else
    lua_pushnumber(L, 0);
  return 2;
}
static int lua_get_special_state(lua_State* L) {
  if (out.stsReg2 & SPECIAL_GAIT_PENDING)
    lua_pushnumber(L, 1);
  else
    lua_pushnumber(L, 0);
  if (out.stsReg2 & GAIT_RESET_PENDING)
    lua_pushnumber(L, 1);
  else
    lua_pushnumber(L, 0);
  if (out.stsReg2 & RESET_ODOMETER_PENDING)
    lua_pushnumber(L, 1);
  else
    lua_pushnumber(L, 0);
  if (out.stsReg2 & WALK_KICK_PENDING)
    lua_pushnumber(L, 1);
  else
    lua_pushnumber(L, 0);
  return 4;
}
static int lua_get_velocity(lua_State* L) {
  int reg = 0;
  lua_pushnumber(L, out.dirSts.xOffset);
  reg++;
  lua_pushnumber(L, out.dirSts.yOffset);
  reg++;
  lua_pushnumber(L, out.dirSts.thetaOffset);
  reg++;
  return reg;
}
static int lua_get_headpos(lua_State* L) {
  int reg = 0;
  lua_pushnumber(L, out.headSts.pitch);
  reg++;
  lua_pushnumber(L, out.headSts.yaw);
  reg++;
  return reg;
}
static int lua_get_odometry(lua_State* L) {
  int reg = 0;
  lua_pushnumber(L, out.odometer.xOffset);
  reg++;
  lua_pushnumber(L, out.odometer.yOffset);
  reg++;
  lua_pushnumber(L, out.odometer.thetaOffset);
  reg++;
  return reg;
}
// b51
static int lua_get_imu_angle(lua_State* L) {
  int reg = 0;
  lua_pushnumber(L, out.sensors.incline[0]);
  reg++;
  lua_pushnumber(L, out.sensors.incline[1]);
  reg++;
  lua_pushnumber(L, out.sensors.incline[2]);
  reg++;
  return reg;
}
// b51
static int lua_get_body_pos(lua_State* L) {
  lua_pushnumber(L, out.isLeft);
  lua_pushnumber(L, out.torsoPose.offset.x);
  lua_pushnumber(L, out.torsoPose.offset.y);
  lua_pushnumber(L, out.torsoPose.offset.z);
  lua_pushnumber(L, out.torsoPose.pose.alpha);
  lua_pushnumber(L, out.torsoPose.pose.beta);
  lua_pushnumber(L, out.torsoPose.pose.theta);
  return 7;
}
static int lua_dcm_entry(lua_State* /*L*/) {
  BeginDspThread();
  printf("dsp in\n");
  return 0;
}
static int lua_dcm_exit(lua_State* /*L*/) {
  destroy_rs232();
  printf("dsp out\n");
  return 0;
}
static int lua_one_time_dsp_thread(lua_State* /*L*/) {
  DspThread();
  return 1;
}
static const struct luaL_Reg dspcode_functions[] = {
    {"pass_state", lua_pass_state},
    {"pass_para", lua_pass_para},
    {"get_gaitValid", lua_get_gdv_from_c},
    {"get_specialValid", lua_get_sgv_from_c},
    {"get_gaitReset", lua_get_grv_from_c},
    {"get_odometerReset", lua_get_cro_from_c},
    {"get_headValid", lua_get_hmv_from_c},
    {"get_torqueEnable", lua_get_tev_from_c},
    {"get_sensorEnable", lua_get_sev_from_c},
    {"get_walkkick", lua_get_walk_kick},
    {"get_velocity", lua_get_velocity},
    {"get_headpos", lua_get_headpos},
    {"get_imuAngle", lua_get_imu_angle},  // b51
    {"get_odometry", lua_get_odometry},
    {"enter_entry", lua_dcm_entry},
    {"dsp_exit", lua_dcm_exit},
    {"get_bodypos", lua_get_body_pos},
    {"dsp_thread", lua_one_time_dsp_thread},
    {"get_pendingstate", lua_get_special_state},
    {NULL, NULL}};

static const struct luaL_Reg dspcode_methods[] = {{NULL, NULL}};

extern "C" int luaopen_DspPacket(lua_State* L) {
  luaL_newmetatable(L, "dsppacket_mt");

  // OO access: mt.__index = mt
  // Not compatible with array access
  lua_pushvalue(L, -1);
  lua_setfield(L, -2, "__index");

  luaL_setfuncs(L, dspcode_methods, 0);
  luaL_setfuncs(L, dspcode_functions, 0);
  return 1;
}
