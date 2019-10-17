/*************************************************************************
 *
 *              Author: b51
 *                Mail: b51live@gmail.com
 *            FileName: DspCmd.h
 *
 *          Created On: Fri Oct  4 23:41:54 2019
 *     Licensed under The MIT License [see LICENSE for details]
 *
 ************************************************************************/

#ifndef DSP_CMD_H_
#define DSP_CMD_H_

#define MAX_PARAM_NUM 255

/**
 *  DSP_INST_PACKET: Instruction packet
 *
 *  id: Dynamixel ID
 *  length: Packet length = parameter's number + 1(ID) + 1(Instruction)
 *  instruction: Instruction code
 *  parameters: Parameters
 */
typedef struct {
  unsigned char id;
  unsigned char length;
  unsigned char instruction;
  unsigned char parameter[MAX_PARAM_NUM];
} DSP_INST_PACKET;

/**
 *  DSP_STATUS_PACKET: Status packet
 *
 *  id: Dynamixel ID
 *  length: Parameter's number + 1(ID) + 1(infomation)
 *  information: Error code
 *  parameters: Parameters
 */
typedef struct {
  unsigned char id;
  unsigned char length;
  unsigned char infomation;                // Error code
  unsigned char parameter[MAX_PARAM_NUM];  // Parameters
} DSP_STATUS_PACKET;

/**
 * Instruction Macros
 */
#define INST_CONNECTION_VALID 0x01  // connection instruction
#define INST_SINGLE_ACTION 0x02     // Read instruction
#define INST_MULTIPLE_ACTION 0x03   // Write instruction
#define INST_PROPERTY_SETUP 0x04    // Reg_write instruction
#define INST_BULK_DOWNLOAD 0x05     // Action instruction
#define INST_RESET 0x06             // Reset instruction
#define INST_STATE_FEEDBACK 0x07    // Sync_write instruction
#define INST_TORQUE_ON 0x08
#define INST_TORQUE_OFF 0x09
#define INST_INITIAL_STATE 0x0a
#define INST_GAIT_MEMORY_START 0x0b
#define INST_ADD_SINGLE_GAIT 0x0c
#define INST_GAIT_COMMAND 0x0d
#define INST_FLASH_PROGRAM 0x0e
#define INST_START_INCLINOMETER_FEEDBACK 0x0f
#define INST_STOP_INCLINOMETER_FEEDBACK 0x10
#define INST_INCLINOMETER_REQUIRED 0x11
#define INST_STOP_GAIT_EXECUTING 0x12
#define INST_GAIT_DIRECTION 0x13
#define INST_GAIT_DIRECTION_EXT 0x14
#define INST_ADD_DATA_PATCH 0x15
#define INST_STATE_SWAP 0x16

#define INFO_GAIT_EXECUTED 0x71
#define INFO_INCLINOMETER_FEEDBACK 0x72
#define INFO_GAIT_EXECUTING 0x73
#define INFO_GAIT_FEEDBACK 0x74

#define PACKET_TYPE_MASK 0x80

#define LOOP_EXECUTE 0xff
#define STOP_EXECUTE 0x00
#define ID_DSP 0xfe

/**
 *  SpecialGaitCommand
 *  id: Special gait id
 *  times: Repeat times
 */
struct SpecialGaitCommand {
  unsigned short id;
  unsigned short times;
};

/**
 *  HeadMovingCommand
 *  pitch: pitch to move, rad * 512
 *  yaw: yaw to move, rad * 512
 */
struct HeadMovingCommand {
  short pitch;
  short yaw;
};

/**
 *  GaitEffect
 *  xOffset: velocity at x direction, m/s * 1000
 *  yOffset: velocity at y direction, m/s * 1000
 *  thetaOffset: velocity at yaw, rad/s * 512
 */
struct GaitEffect {
  short xOffset;
  short yOffset;
  short thetaOffset;
};

/**
 *  Sensors
 *  IMU Angles, [0]:roll, [1]:pitch, [2]:yaw, value / 512 = rad
 *  pitch > 0 robot face to ground
 */
struct Sensors {
  short incline[3];
};

/**
 *  StateSwapInput
 *  States Input To Dsp
 *  ctrReg1: some flags to dsp
 *           SENSOR_ENABLE_VALID, COPY_RESET_ODOMETER, SPECIAL_GAIT_VALID
 *           WALK_KICK_LEFT, WALK_KICK_RIGHT, TORQUE_ENABLE_VALID,
 *           GAIT_DIRECTION_VALID, GAIT_RESET_VALID
 *  ctrReg2: some flags from dsp
 *           SPECIAL_GAIT_PENDING, GAIT_RESET_PENDING,
 *           RESET_ODOMETER_PENDING, WALK_KICK_PENDING
 *  dirInst: velocity input to dsp
 *  spcInst: special gait input to dsp
 *  headInst: head moving command to dsp
 */
struct StateSwapInput {
  unsigned short ctrReg1;
  unsigned short ctrReg2;
  struct GaitEffect dirInst;
  struct SpecialGaitCommand spcInst;
  struct HeadMovingCommand headInst;
};

#define NUMSTATE 10
#define NUMPARA 10
#define GAIT_DIRECTION_VALID 0x0001
#define SPECIAL_GAIT_VALID 0x0002
#define GAIT_RESET_VALID 0x0004
#define COPY_RESET_ODOMETER 0x0008
#define HEAD_MOVE_VALID 0x0010
#define TORQUE_ENABLE_VALID 0x0020
#define SENSOR_ENABLE_VALID 0x0040
#define WALK_KICK_LEFT 0x0080
#define WALK_KICK_RIGHT 0x0100
#define SPECIAL_GAIT_PENDING 0x0001
#define GAIT_RESET_PENDING 0x0002
#define RESET_ODOMETER_PENDING 0x0004
#define WALK_KICK_PENDING 0x0008  // walkkick

struct RigidOffset {
  short x;
  short y;
  short z;
};

struct RigidPose {
  short alpha;
  short beta;
  short theta;
};
struct RigidBody {
  struct RigidOffset offset;
  struct RigidPose pose;
};

/**
 *  structCArray
 *  For communication with lua
 */
typedef struct {
  const double* ptr;
  char type;
  int size;
  int own;  // 1 if array was created by Lua and needs to be deleted
} structCArray;
structCArray *stateIn[NUMSTATE], *paraIn[NUMPARA];

char *stateName[NUMSTATE], *paraName[NUMPARA];
static int stateNum = 0, paraNum = 0;

/**
 *  StateSwapOutput
 *  States Get Out From dsp
 *  stsReg1: some flags from dsp
 *           GAIT_DIRECTION_VALID, SPECIAL_GAIT_VALID, GAIT_RESET_VALID,
 *           COPY_RESET_ODOMETER, HEAD_MOVE_VALID, TORQUE_ENABLE_VALID,
 *           SENSOR_ENABLE_VALID, WALK_KICK_LEFT, WALK_KICK_RIGHT
 *  stsReg2: some special gait state from dsp
 *           SPECIAL_GAIT_PENDING, GAIT_RESET_PENDING,
 *           RESET_ODOMETER_PENDING, WALK_KICK_PENDING
 *  dirSts:  get velocity from dsp
 *  spcSts:
 *  headSts: get current head pose from dsp, pitch and yaw
 *  odometer: get odometry data, diff data from last odometry
 *            当前里程计对上一次更新值的相对位移
 *  sensors: get current IMU data from dsp
 *  isLeft: is left foot ahead (not sure)
 *  torsoPose: get current robot body pose
 */
struct StateSwapOutput {
  unsigned short stsReg1;
  unsigned short stsReg2;
  struct GaitEffect dirSts;
  struct SpecialGaitCommand spcSts;
  struct HeadMovingCommand headSts;
  struct GaitEffect odometer;
  struct Sensors sensors;
  unsigned short isLeft;
  struct RigidBody torsoPose;
};

#define MAX_ODOMETER_RECORD 5
#define ODOMETER_RECORD_VALID 0x0001
struct OdometerRecord {
  unsigned int flag;
  struct GaitEffect lastRecord;
};

// communiction related static data

// instruction related static data
struct StateSwapInput in;
struct StateSwapOutput out;
void (*pGetNextFrame)(short* ppitch, short* pyaw);
struct OdometerRecord odometers[MAX_ODOMETER_RECORD];

/*
bool DestroyDspCode(void);
bool InitDspCode(void);

bool MoveHead(int pitch, int yaw);
bool InitContinueousHeadMoving(void (*pFun)(PSHORT ppitch, PSHORT pyaw));
bool EnableSensorFeedback(BOOL enable);
bool EnableTorque(BOOL enable);
bool GaitReset();
bool ExecuteGait(int id, int times);
bool ExecuteSpeed(int vx, int vy, int omega);

int OpenOdometer();
bool ReadOdometer(int id, int* pdeltax, int* pdeltay, int* pdeltatheta);
bool CloseOdometer(int id);

bool GetHeadMatrix(int* ppitch, int* pyaw);
bool GetSensorFeedBack(int* proll, int* ppitch);
bool IsSensorFeedbackEnabled();
bool IsTorqueEnabled();
bool GetRemainningGait(int* pid, int* ptimes);
bool GetSpeed(int* pvx, int* pvy, int* pomega);
void EnterPreparingState(void);
void BeginDspThread(void);
void DspThread(void);
void setStateToIn(void);
void setParaToIn(void);
*/

#endif
