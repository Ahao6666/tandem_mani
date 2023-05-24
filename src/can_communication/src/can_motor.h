#include <iostream>
#include <cstring>
#include <unistd.h>
#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <socketcan_interface/socketcan.h>

#define PBUF_SIZE	9 * 0x100

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
	uint8_t	canRxData[9];
}mcProtocol_t;

void MC_ProtocolEncode(uint8_t index, uint8_t mode, uint8_t cmd, uint8_t* buf);
void MC_FindOnlineMotor(void);
void MC_HeartbeatEncode(uint8_t index);
uint8_t MC_GetOnlineMotorNum(void);
uint8_t MC_GetMotorNum(uint8_t index);


uint8_t canDataToSend[6];		 /* 发送数据缓存 */