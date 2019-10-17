#ifndef DSP_CONTROL_H
#define DSP_CONTROL_H

/**************** Data structure ************************/
#define MAX_PARAM_NUM		255			// Maximum parameter number

////////////// Instruction packet ////////////////////////
typedef struct
{
	BYTE	id;							// Dynamixel ID
	BYTE	length;				// Packet length = parameter's number + 1(ID) + 1(Instruction)
	BYTE	instruction;				// Instruction code
	BYTE	parameter[MAX_PARAM_NUM];	// Parameters
}DSP_INST_PACKET;

////////////////// Status packet /////////////////////////
typedef struct
{
	BYTE	id;							// Dynamixel ID
	BYTE	length;				// Packet length = parameter's number + 1(ID) + 1(infomation)
	BYTE	infomation;						// Error code
	BYTE	parameter[MAX_PARAM_NUM];	// Parameters
}DSP_STATUS_PACKET;

/****************** Define Macro ************************/

///////////////// Instruction code ///////////////////////
#define INST_CONNECTION_VALID           0x01		// connection instruction
#define INST_SINGLE_ACTION				0x02		// Read instruction
#define INST_MULTIPLE_ACTION			0x03		// Write instruction
#define INST_PROPERTY_SETUP				0x04		// Reg_write instruction
#define INST_BULK_DOWNLOAD				0x05		// Action instruction
#define INST_RESET						0x06		// Reset instruction
#define INST_STATE_FEEDBACK				0x07		// Sync_write instruction
#define INST_TORQUE_ON					0x08
#define INST_TORQUE_OFF					0x09
#define INST_INITIAL_STATE				0x0a
#define INST_GAIT_MEMORY_START			0x0b
#define INST_ADD_SINGLE_GAIT			0x0c
#define INST_GAIT_COMMAND				0x0d
#define INST_FLASH_PROGRAM				0x0e
#define INST_START_INCLINOMETER_FEEDBACK	0x0f
#define INST_STOP_INCLINOMETER_FEEDBACK		0x10
#define INST_INCLINOMETER_REQUIRED			0x11
#define INST_STOP_GAIT_EXECUTING			0x12
#define INST_GAIT_DIRECTION                 0x13
#define INST_GAIT_DIRECTION_EXT				0x14
#define INST_ADD_DATA_PATCH					0x15
#define INST_STATE_SWAP						0x16

#define INFO_GAIT_EXECUTED				0x71
#define INFO_INCLINOMETER_FEEDBACK		0x72
#define INFO_GAIT_EXECUTING				0x73
#define INFO_GAIT_FEEDBACK              0x74

#define PACKET_TYPE_MASK				0x80

#define LOOP_EXECUTE					0xff
#define STOP_EXECUTE					0x00
#define ID_DSP							0xfe
#endif
