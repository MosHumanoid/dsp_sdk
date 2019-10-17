#ifndef DSP_CODE_H
#define DSP_CODE_H

#include "linuxdef.h"
#include "DspControl.h"

struct SpecialGaitCommand
{
	USHORT id;//²œÌ¬id
	USHORT times;//ÖŽÐÐŽÎÊý
};

struct HeadMovingCommand//Í·²¿µÄÊµŒÊ×ªœÇ
{
	SHORT pitch;
	SHORT yaw;
};

struct GaitEffect
{
	SHORT xOffset;
	SHORT yOffset;
	SHORT thetaOffset;
};

struct Sensors
{
	SHORT incline[3];
};

struct StateSwapInput
{
	USHORT ctrReg1;
	USHORT ctrReg2;
	struct GaitEffect dirInst;
	struct SpecialGaitCommand spcInst;
	struct HeadMovingCommand headInst;
};
#define NUMSTATE    10
#define NUMPARA     10
#define GAIT_DIRECTION_VALID		0x0001//ËÙ¶È
#define SPECIAL_GAIT_VALID		0x0002//ÌØÊâ²œÌ¬
#define GAIT_RESET_VALID		0x0004//ÔË¶¯žŽÎ»
#define COPY_RESET_ODOMETER		0x0008//Â·³ÌŒÆÇåÁã£¬ÔÝÊ±²»ÓÃ
#define HEAD_MOVE_VALID			0x0010//Í·²¿ÔË¶¯ÖžÁîÓÐÐ§
#define TORQUE_ENABLE_VALID		0x0020//¶æ»úžøÁŠ
#define SENSOR_ENABLE_VALID		0x0040//ÓÀÔ¶ÖÃ1
#define WALK_KICK_LEFT			0x0080
#define WALK_KICK_RIGHT			0x0100
#define SPECIAL_GAIT_PENDING		0x0001
#define GAIT_RESET_PENDING		0x0002
#define RESET_ODOMETER_PENDING		0x0004
#define WALK_KICK_PENDING		0x0008
//walkkick

struct RigidOffset
{
	SHORT x;
	SHORT y;
	SHORT z;
};

struct RigidPose
{
	SHORT alpha;
	SHORT beta;
	SHORT theta;
};
struct RigidBody
{
	struct RigidOffset offset;
	struct RigidPose pose;
};
typedef struct {
  const double *ptr;
  char type;
  int size;
  int own; // 1 if array was created by Lua and needs to be deleted
} structCArray;
structCArray *stateIn[NUMSTATE],*paraIn[NUMPARA];

char *stateName[NUMSTATE],*paraName[NUMPARA];
static int stateNum=0,paraNum=0;
struct StateSwapOutput//ÖŽÐÐÍêÁËÖÃ³É0£¬·µ»ØµÄ¶«Î÷²ð¿ª£¬·¢ÏÂµÄ¶«Î÷Žò°ü
{
	USHORT stsReg1;
	USHORT stsReg2;
	struct GaitEffect dirSts;//·µ»ØµÄÊµŒÊÊýŸÝ£¬žúÖžÁîµÄ¹²ÏíÄÚŽæ·Ö¿ª
	struct SpecialGaitCommand spcSts;//Ê£Óà¶àÉÙ²œÌ¬
	struct HeadMovingCommand headSts;//Í·²¿µÄÊµŒÊ×ªœÇ
	struct GaitEffect odometer;//ÅÜµÄÂ·³Ì
	struct Sensors sensors;//
        USHORT isLeft;
	struct RigidBody torsoPose;
};

#define MAX_ODOMETER_RECORD	5
#define ODOMETER_RECORD_VALID	0x0001
struct OdometerRecord
{
	UINT flag;
	struct GaitEffect lastRecord;
};

//communiction related static data

CHAR packetBuffer[sizeof(DSP_INST_PACKET)+2];
CHAR packetToProc[sizeof(DSP_INST_PACKET)+2];


//instruction related static data
struct StateSwapInput in;
struct StateSwapOutput out;
void (*pGetNextFrame)(PSHORT ppitch,PSHORT pyaw);
struct OdometerRecord odometers[MAX_ODOMETER_RECORD];




BOOL DestroyDspCode(void);
BOOL InitDspCode(void);

BOOL MoveHead(int pitch, int yaw);
BOOL InitContinueousHeadMoving(void (*pFun)(PSHORT ppitch,PSHORT pyaw));
BOOL EnableSensorFeedback(BOOL enable);
BOOL EnableTorque(BOOL enable);
BOOL GaitReset();
BOOL ExecuteGait(int id,int times);
BOOL ExecuteSpeed(int vx,int vy,int omega);

int OpenOdometer();
BOOL ReadOdometer(int id,int *pdeltax,int *pdeltay,int *pdeltatheta);
BOOL CloseOdometer(int id);

BOOL GetHeadMatrix(int *ppitch,int *pyaw);
BOOL GetSensorFeedBack(int *proll,int *ppitch);
BOOL IsSensorFeedbackEnabled();
BOOL IsTorqueEnabled();
BOOL GetRemainningGait(int *pid,int *ptimes);
BOOL GetSpeed(int *pvx,int *pvy,int *pomega);
void EnterPreparingState(void);
void BeginDspThread(void);
void DspThread(void);
void setStateToIn(void);
void setParaToIn(void);


#endif
