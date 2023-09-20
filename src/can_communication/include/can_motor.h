#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include <socketcan_interface/socketcan.h>
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include "can_communication/MotorPosTar.h"
#include "can_communication/MotorSpdTar.h"
#include "can_communication/MotorModeSet.h"
#include "can_communication/MotorPIDSet.h"

#define PBUF_SIZE	9 * 0x100
#define HEARTBEAT 	0x01

typedef enum
{
	MODE_HEARTBEAT=1,
	MODE_READ,
	MODE_FASTWRITE,
	MODE_WRITE,
	MODE_REGWRITE,
	MODE_REGREAD,
	MODE_CMDWRITE,
	MODE_RETURN,
}mcProtocolModeDevice_e;
typedef enum
{
	MODEH_READ=0x72,
	MODEH_FASTWRITE,
	MODEH_WRITE,
	MODEH_REGWRITE,
	MODEH_REGREAD,
	MODEH_CMDWRITE,
}mcProtocolModeHost_e;
typedef enum
{
	CMD_ID=0x11,
	CMD_IQ,
	CMD_SPEED,
	CMD_POS
}mcProtocolWrite_e;
typedef enum
{
	REG_ALL = 0,
	REG_KP,
	REG_KI,
	REG_KD,
	REG_CANINDEX
}mcProtocolReg_e;
typedef enum
{
	CMD_STARTMOTOR=1,
	CMD_STOPMOTOR,
	CMD_CLEANERR,
	CMD_RUNMODE,
}mcProtocolCmdWrite_e;
typedef enum
{
	RTORQUE_MODE,
	RSPEED_MODE,
	/* 普通位置模式 */
	RPOSITION_MODE,
	/* 线性插帧模式 */
	RPOSITION_MODE_T
}srunMode_e;
typedef enum
{
	SIDLE,
	SRUN,
	SALIGNMENT,
	SSTART,
	SERROR,
}smotorState_e;

typedef enum
{
	FNO_ERROR,
	F_ERROR_VOLT,		/**< @brief Error: Software over or under voltage.*/
	F_ERROR_CURR,		/**< @brief Error: Over curren.*/
	F_ERROR_SPEED,		/**< @brief Error: Speed feedback.*/
	F_ERROR_TIME,		/**< @brief Error: FOC rate to high.*/
	F_ERRORSW,			/**< @brief Software Error.*/
}smotorFault_e;

typedef struct mcProtocol
{
	uint8_t initFinish;
	uint8_t index; 
	uint32_t tick;
	uint8_t refreshFlag;
	uint8_t findDone;
	float packetLoss;
	uint32_t sTick;
	uint32_t rTick;
	uint8_t connectID[0x100];
	uint8_t connectFlag[0x100];
	uint8_t onlineNum;
	uint8_t	rxBuf[PBUF_SIZE];
	uint8_t	txBuf[PBUF_SIZE];
	uint8_t	canTxData[9];
	uint8_t	canRxData[8];
}mcProtocol_t;

typedef struct mcParamPID
{
	uint16_t kp;
	uint16_t ki;
	uint16_t kd;
}mcProtocolPID_t;

typedef struct mcProtocolMotorData
{
	uint32_t sTick;
	uint32_t rTick;
	uint8_t canIndex;
	uint8_t online;
	int16_t id;
	int16_t iq;
	int16_t speed;
	float pos;
	
	mcProtocolPID_t idPID;
	mcProtocolPID_t iqPID;
	mcProtocolPID_t speedPID;
	mcProtocolPID_t positionPID;
	
	srunMode_e 		runMode;
	smotorState_e 	smotorState;
	smotorFault_e 	motorFault;
	
}mcProtocolMotorData_t;
typedef union u2f
{
	float f;
	unsigned char	c[4];
}u2f_u;
void MC_HeartbeatEncode(uint8_t index);
void MC_FindOnlineMotor(void);
void receiveCallback(const can_msgs::Frame::ConstPtr& msg);
void MC_ProtocolEncode(uint8_t index, uint8_t mode, uint8_t cmd, uint8_t* buf);
static void MC_ReadIdDecode(uint8_t index, uint8_t mode,uint8_t cmd);
static void MC_ReadIqDecode(uint8_t index, uint8_t mode,uint8_t cmd);
static void MC_ReadSpeedDecode(uint8_t index, uint8_t mode,uint8_t cmd);
static void MC_ReadPositionDecode(uint8_t index, uint8_t mode,uint8_t cmd);
uint8_t MC_GetMotorNum(uint8_t index);
void MC_ReadHeartbeatDecode(uint8_t index, uint8_t mode, uint8_t cmd);

