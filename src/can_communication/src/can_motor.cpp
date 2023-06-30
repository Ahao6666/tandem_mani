#include "can_motor.h"

can_msgs::Frame frame_send;
ros::Publisher can_pub;
uint8_t canDataToSend[6];		 /* 发送数据缓存 */
mcProtocol_t mcProtocol;
mcProtocolMotorData_t *motorData = NULL;
can_communication::MotorPosTar MotorPosTar_msg;
typedef void (*MC_DataDecode)(uint8_t, uint8_t, uint8_t );
u2f_u u2f;
#define  BYTE0(dwTemp)       ( *( (uint8_t *)(&dwTemp)	)  )
#define  BYTE1(dwTemp)       ( *( (uint8_t *)(&dwTemp) + 1) )
#define  BYTE2(dwTemp)       ( *( (uint8_t *)(&dwTemp) + 2) )
#define  BYTE3(dwTemp)       ( *( (uint8_t *)(&dwTemp) + 3) )
/* 解析函数向量表1 */
static const MC_DataDecode MC_DecodeTalbe1[0X04] =
{
  MC_ReadIdDecode,
  MC_ReadIqDecode, 
  MC_ReadSpeedDecode,
  MC_ReadPositionDecode, 
};

static const MC_DataDecode MC_DecodeTalbe3[0X04] =
{
  MC_ReadIdDecode,
  MC_ReadIqDecode, 
  MC_ReadSpeedDecode,
  MC_ReadPositionDecode,
};
static void MC_ReadIdDecode(uint8_t num, uint8_t mode ,uint8_t cmd)
{

}

static void MC_ReadIqDecode(uint8_t index, uint8_t mode ,uint8_t cmd)
{
	motorData[MC_GetMotorNum(index)].iq =  (mcProtocol.canRxData[2] | (uint16_t)(mcProtocol.canRxData[3]<<8));
}
static void MC_ReadSpeedDecode(uint8_t index, uint8_t mode,uint8_t cmd)
{
	motorData[MC_GetMotorNum(index)].speed =  (mcProtocol.canRxData[2] | (uint16_t)(mcProtocol.canRxData[3]<<8));
}
static void MC_ReadPositionDecode(uint8_t index, uint8_t mode,uint8_t cmd)
{
	u2f.c[0] = mcProtocol.canRxData[2];
	u2f.c[1] = mcProtocol.canRxData[3];
	u2f.c[2] = mcProtocol.canRxData[4];
	u2f.c[3] = mcProtocol.canRxData[5];
	motorData[MC_GetMotorNum(index)].pos = u2f.f;
}

uint8_t MC_GetMotorNum(uint8_t index)
{
	for(uint8_t i = 0; i<mcProtocol.onlineNum; i++)
	{
		if(index ==mcProtocol.connectID[i] )
		{
			return i;
		}
	}
	return 0;
}
void MC_FindOnlineMotor(void)
{
	mcProtocol.refreshFlag = 1;
	for(uint8_t i = 0;i<0x10; i++ )
	{
		MC_HeartbeatEncode(i);
		/* connect clean */
		mcProtocol.connectFlag[i] = 0;
	}
}

void MC_ProtocolSendLoop(void)	
{
	if(mcProtocol.findDone == 1)
	{
		/* beatheart all can device */
		for(uint8_t i = 0; i<mcProtocol.onlineNum; i++)
		{
			motorData[i].sTick++;
			mcProtocol.sTick++;
			MC_HeartbeatEncode(motorData[i].canIndex);
		}
	}
}

void MC_HeartbeatEncode(uint8_t index)
{
	memset(canDataToSend, 0, sizeof(canDataToSend));
	MC_ProtocolEncode(index, MODE_HEARTBEAT , 0x01 , canDataToSend);

}

void MC_ReadIdEncode(uint8_t index)
{
	
}

void MC_ReadIqEncode(uint8_t num)
{
	memset(canDataToSend, 0, sizeof(canDataToSend));
	MC_ProtocolEncode(motorData[num].canIndex, MODE_READ, CMD_IQ, canDataToSend);

}
void MC_ReadSpeedEncode(uint8_t num)
{
	memset(canDataToSend, 0, sizeof(canDataToSend));
	MC_ProtocolEncode(motorData[num].canIndex, MODE_READ, CMD_SPEED, canDataToSend);	
}
void MC_ReadPositionEncode(uint8_t num)
{
	memset(canDataToSend, 0, sizeof(canDataToSend));
	MC_ProtocolEncode(motorData[num].canIndex, MODE_READ, CMD_POS, canDataToSend);		
}
void MC_WriteIdEncode(uint8_t index)
{
	memset(canDataToSend, 0, sizeof(canDataToSend));
	
}
void MC_WriteIqEncode(uint8_t num, int16_t iq, uint16_t fDuration)
{
	memset(canDataToSend, 0, sizeof(canDataToSend));
	canDataToSend[2] = BYTE0(iq);
	canDataToSend[3] = BYTE1(iq);
	canDataToSend[4] = BYTE0(fDuration);
	canDataToSend[5] = BYTE1(fDuration);
	MC_ProtocolEncode(motorData[num].canIndex, MODE_WRITE, CMD_IQ, canDataToSend);
}
void MC_WriteSpeedEncode(uint8_t num, int16_t speed, uint16_t fDuration)
{
	memset(canDataToSend, 0, sizeof(canDataToSend));
	canDataToSend[2] = BYTE0(speed);
	canDataToSend[3] = BYTE1(speed);
	canDataToSend[4] = BYTE0(fDuration);
	canDataToSend[5] = BYTE1(fDuration);
	MC_ProtocolEncode(motorData[num].canIndex, MODE_WRITE, CMD_SPEED, canDataToSend);
}

void MC_WritePositionEncode(uint8_t num, float position, uint16_t fDuration)
{
	memset(canDataToSend, 0, sizeof(canDataToSend));
	canDataToSend[0] = BYTE0(position);
	canDataToSend[1] = BYTE1(position);
	canDataToSend[2] = BYTE2(position);
	canDataToSend[3] = BYTE3(position);
	canDataToSend[4] = BYTE0(fDuration);
	canDataToSend[5] = BYTE1(fDuration);
	MC_ProtocolEncode(motorData[num].canIndex, MODE_WRITE, CMD_POS, canDataToSend);
}

void MC_FastWriteIqEncode(uint8_t num, int16_t iq, uint16_t fDuration)
{
	memset(canDataToSend, 0, sizeof(canDataToSend));
	canDataToSend[2] = BYTE0(iq);
	canDataToSend[3] = BYTE1(iq);
	canDataToSend[4] = BYTE0(fDuration);
	canDataToSend[5] = BYTE1(fDuration);
	MC_ProtocolEncode(motorData[num].canIndex, MODE_FASTWRITE, CMD_IQ, canDataToSend);
}
void MC_FastWriteSpeedEncode(uint8_t num, int16_t speed, uint16_t fDuration)
{
	memset(canDataToSend, 0, sizeof(canDataToSend));
	canDataToSend[2] = BYTE0(speed);
	canDataToSend[3] = BYTE1(speed);
	canDataToSend[4] = BYTE0(fDuration);
	canDataToSend[5] = BYTE1(fDuration);
	MC_ProtocolEncode(motorData[num].canIndex, MODE_FASTWRITE, CMD_SPEED, canDataToSend);
}

void MC_FastWritePositionEncode(uint8_t num, float position, uint16_t fDuration)
{
	memset(canDataToSend, 0, sizeof(canDataToSend));
	canDataToSend[0] = BYTE0(position);
	canDataToSend[1] = BYTE1(position);
	canDataToSend[2] = BYTE2(position);
	canDataToSend[3] = BYTE3(position);
	canDataToSend[4] = BYTE0(fDuration);
	canDataToSend[5] = BYTE1(fDuration);
	MC_ProtocolEncode(motorData[num].canIndex, MODE_FASTWRITE, CMD_POS, canDataToSend);
}

void MC_StartMotorEncode(uint8_t num)
{
	memset(canDataToSend, 0, sizeof(canDataToSend));
	MC_ProtocolEncode(motorData[num].canIndex, MODE_CMDWRITE, CMD_STARTMOTOR, canDataToSend);
}

void MC_StopMotorEncode(uint8_t num)
{
	memset(canDataToSend, 0, sizeof(canDataToSend));
	MC_ProtocolEncode(motorData[num].canIndex, MODE_CMDWRITE, CMD_STOPMOTOR, canDataToSend);
}

void MC_CleanMotorEncode(uint8_t num)
{
	memset(canDataToSend, 0, sizeof(canDataToSend));
	MC_ProtocolEncode(motorData[num].canIndex, MODE_CMDWRITE, CMD_CLEANERR, canDataToSend);
}

void MC_SetMotorModeEncode(uint8_t num , uint8_t mode)
{
	memset(canDataToSend, 0, sizeof(canDataToSend));
	canDataToSend[0] = mode;
	MC_ProtocolEncode(motorData[num].canIndex, MODE_CMDWRITE, CMD_RUNMODE, canDataToSend);
}

void MC_SetPID_Encode(uint8_t num, uint8_t type, uint8_t pid, int16_t data1, uint8_t saveFlag)
{
	memset(canDataToSend, 0, sizeof(canDataToSend));
	canDataToSend[0] = saveFlag;
	canDataToSend[1] = 0;
	canDataToSend[2] = 0;
	canDataToSend[3] = type;
	canDataToSend[4] = BYTE0(data1);
	canDataToSend[5] = BYTE1(data1);
	MC_ProtocolEncode(motorData[num].canIndex, MODE_REGWRITE, pid, canDataToSend);	
}

void MC_SetCanIndexEncode(uint8_t num , uint8_t index)
{
	memset(canDataToSend, 0, sizeof(canDataToSend));
	canDataToSend[3] = index;
	MC_ProtocolEncode(motorData[num].canIndex, MODE_REGWRITE, REG_CANINDEX, canDataToSend);
}

void MC_SetAllSaveEncode(uint8_t num)
{
	memset(canDataToSend, 0, sizeof(canDataToSend));
	canDataToSend[0] = 0x01;	
	MC_ProtocolEncode(motorData[num].canIndex, MODE_REGWRITE, REG_ALL, canDataToSend);
}

void MC_ProtocolReturn(uint8_t index, uint8_t mode, uint8_t cmd, uint8_t flag)
{

}
void MC_ProtocolEncode(uint8_t index, uint8_t mode, uint8_t cmd, uint8_t* buf)
{
  // 创建一个CAN帧
  frame_send.header.frame_id = "can_frame";
  frame_send.header.stamp = ros::Time::now();
  frame_send.is_extended = false;
  frame_send.is_error = false;
  frame_send.dlc = 8;  // 设置数据长度
  frame_send.id = index;  // 设置CAN帧的ID
  // 设置CAN帧的数据
  frame_send.data[0] = mode;
  frame_send.data[1] = cmd;
  frame_send.data[2] = buf[0];
  frame_send.data[3] = buf[1];
  frame_send.data[4] = buf[2];
  frame_send.data[5] = buf[3];
  frame_send.data[6] = buf[4];
  frame_send.data[7] = buf[5];
  can_pub.publish(frame_send);
}

void MC_ProtocolInit(void)
{
	for(uint8_t i = 0; i<0x10; i++)
	{
		mcProtocol.connectFlag[i] = 0;
		mcProtocol.connectID[i] = 0;
	}
	mcProtocol.onlineNum = 0;
	mcProtocol.index = 0;
	mcProtocol.initFinish = 1;
}
//电机控制协议解析
void MC_ProtocolDecode(void)
{
	uint8_t connectNum = 0;
  switch (mcProtocol.canRxData[0])
  {
    case HEARTBEAT:
      MC_ReadHeartbeatDecode(mcProtocol.index,mcProtocol.canRxData[0],mcProtocol.canRxData[1]);
      break;
    case MODEH_READ:
      MC_DecodeTalbe1[mcProtocol.canRxData[1]-0x11](mcProtocol.index, MODE_READ, mcProtocol.canRxData[1]);			
      break;
    case MODEH_WRITE:
      MC_DecodeTalbe3[mcProtocol.canRxData[1]-0x11](mcProtocol.index, MODEH_WRITE, mcProtocol.canRxData[1]);				
      break;
    case MODEH_REGWRITE:

      break;
    case MODEH_REGREAD:

      break;
    case MODEH_CMDWRITE:

      break;
    default:
      break;
  }
	if(mcProtocol.refreshFlag == 1)
	{
		mcProtocol.tick++;
		if(mcProtocol.tick % 10 == 0)
		{
			mcProtocol.refreshFlag = 0;
			connectNum = 0;

			for(uint8_t i = 0; i<0x10; i++)
			{
				if(mcProtocol.connectFlag[i] == 1)
				{
					/* mark the can id */
					mcProtocol.connectID[connectNum] = i;
					connectNum++;
				}
			}
			if(motorData != NULL)
			{
				free(motorData);
			}
			/* malloc the num of canid */
			motorData = (mcProtocolMotorData_t *)malloc(sizeof(mcProtocolMotorData_t) * connectNum);
			mcProtocol.onlineNum = connectNum;
			/* distribute can id to struct mcProtocolMotorData_t */
			for(uint8_t i = 0; i<mcProtocol.onlineNum; i++)
			{
				motorData[i].canIndex = mcProtocol.connectID[i];
				motorData[i].online = 1;
			}
			mcProtocol.findDone = 1;
		}
	}
}
uint8_t MC_GetOnlineMotorNum(void)
{
	return mcProtocol.onlineNum;
}
mcProtocolMotorData_t MC_GetMotorData(uint8_t i)
{
	if(i > mcProtocol.onlineNum - 1)
	{
		return motorData[0];
	}
	else
	{
		return motorData[i];
	}
}
void MC_ReadHeartbeatDecode(uint8_t index, uint8_t mode, uint8_t cmd)
{
	mcProtocol.connectFlag[index] = 1;
	if(mcProtocol.findDone == 1)
	{
		motorData[MC_GetMotorNum(index)].rTick++;
		mcProtocol.rTick++;
		mcProtocol.packetLoss = 1.0f - (float)((float)mcProtocol.rTick / mcProtocol.sTick);
		motorData[MC_GetMotorNum(index)].smotorState = (smotorState_e)mcProtocol.canRxData[2];
		motorData[MC_GetMotorNum(index)].runMode = (srunMode_e)mcProtocol.canRxData[3];
		motorData[MC_GetMotorNum(index)].motorFault = (smotorFault_e)mcProtocol.canRxData[4];
	}
}

void Heartbeat_Callback(const ros::TimerEvent& e){
  if(MC_GetOnlineMotorNum() != 0)
    for(int i=0; i<MC_GetOnlineMotorNum(); i++){
      MC_HeartbeatEncode(i);
    }
}
void receiveCallback(const can_msgs::Frame::ConstPtr& msg)
{
  mcProtocol.index = msg->id;
  mcProtocol.canRxData[0] = msg->data[0];
  mcProtocol.canRxData[1] = msg->data[1];
  mcProtocol.canRxData[2] = msg->data[2];
  mcProtocol.canRxData[3] = msg->data[3];
  mcProtocol.canRxData[4] = msg->data[4];
  mcProtocol.canRxData[5] = msg->data[5];
  mcProtocol.canRxData[6] = msg->data[6];
  mcProtocol.canRxData[7] = msg->data[7];

  MC_ProtocolDecode();
  // 处理接收到的CAN帧数据
  // ROS_INFO("Received CAN frame - ID: 0x%02X, Data: %02X %02X %02X %02X %02X %02X %02X %02X",
  //          msg->id, msg->data[0], msg->data[1], msg->data[2], msg->data[3],
  //          msg->data[4], msg->data[5], msg->data[6], msg->data[7]);
}
void MotorPosTar_Callback(const can_communication::MotorPosTar& msg)
{
	MotorPosTar_msg.num = msg.num;
	MotorPosTar_msg.pos_tar = msg.pos_tar;
	MotorPosTar_msg.time_dur = msg.time_dur;
}
int main(int argc, char** argv)
{
  // 初始化ROS节点
  ros::init(argc, argv, "can_motor_node");
  ros::NodeHandle nh;

  const char* ifname = "can0"; // CAN接口名，根据实际情况修改
  int sockfd;
  // 创建套接字
  sockfd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (sockfd == -1) {
      std::cerr << "Failed to create socket." << std::endl;
      return 1;
  }
  // 绑定到CAN接口
  struct ifreq ifr;
  std::strcpy(ifr.ifr_name, ifname);
  if (ioctl(sockfd, SIOCGIFINDEX, &ifr) == -1) {
      std::cerr << "Failed to get interface index." << std::endl;
      close(sockfd);
      return 1;
  }
  struct sockaddr_can addr;
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(sockfd, (struct sockaddr*)&addr, sizeof(addr)) == -1) {
      std::cerr << "Failed to bind socket to interface." << std::endl;
      close(sockfd);
      return 1;
  }

  // 创建CAN消息发布者
  can_pub = nh.advertise<can_msgs::Frame>("sent_messages", 15);

  // 创建ros 定时器
  ros::Timer Heartbeat_timer = nh.createTimer(ros::Duration(0.2), Heartbeat_Callback);   //100Hz

  // 创建CAN消息订阅者
  ros::Subscriber can_sub = nh.subscribe("received_messages", 10, receiveCallback);
  // 接收电机位置参考信息
  ros::Subscriber MotorPosTar_sub = nh.subscribe("MotorPosTar", 10, MotorPosTar_Callback);
  MotorPosTar_msg.pos_tar = {0, 0, 0, 0, 0};
  MotorPosTar_msg.time_dur = {0, 100, 100, 100, 100};
  MC_ProtocolInit();

  // 发布和订阅CAN消息
  while (ros::ok())
  {
    std::cout<< "OnlineMotorNum is:"<<unsigned(MC_GetOnlineMotorNum()) << std::endl;
    
    if(MC_GetOnlineMotorNum() == 0){
      MC_FindOnlineMotor();
      // MC_StopMotorEncode(0);
    }
    else{
      for(int i=0; i<MC_GetOnlineMotorNum(); i++){
        // If the motor is idle, then start
        if(!motorData[MC_GetMotorNum(i)].smotorState)
          MC_StartMotorEncode(i);
        MC_WritePositionEncode(i, MotorPosTar_msg.pos_tar[i], MotorPosTar_msg.time_dur[i]);
      }
    }

    ros::Duration(0.1).sleep();
    
    // 处理订阅的消息
    ros::spinOnce();
  }

  return 0;
}
