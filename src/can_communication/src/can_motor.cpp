#include "can_motor.h"

int sockfd;
can_msgs::Frame frame_send;
can_msgs::Frame frame_rece;
ros::Publisher can_pub;
uint8_t canDataToSend[6];		 /* 发送数据缓存 */
mcProtocol_t mcProtocol;
mcProtocolMotorData_t *motorData = NULL;
typedef void (*MC_DataDecode)(uint8_t, uint8_t, uint8_t );
u2f_u u2f;

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
	for(uint8_t i = 0; i<0x05; i++)
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
  // std::cout<<"11111111"<<std::endl;
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
    std::cout<<mcProtocol.tick<<std::endl;
		mcProtocol.tick++;
		if(mcProtocol.tick % 10 == 0)
		{
			mcProtocol.refreshFlag = 0;
			connectNum = 0;

			for(uint8_t i = 0; i<0x30; i++)
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
void MC_HeartbeatEncode(uint8_t index)
{
	memset(canDataToSend, 0, sizeof(canDataToSend));
	MC_ProtocolEncode(index, MODE_HEARTBEAT , 0x01 , canDataToSend);
}
void MC_FindOnlineMotor(void)
{
  mcProtocol.refreshFlag = 1;
	for(uint8_t i = 0; i<0x05; i++ )
	{
		MC_HeartbeatEncode(i);
		/* connect clean */
		mcProtocol.connectFlag[i] = 0;
	}
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
  ROS_INFO("Received CAN frame - ID: 0x%03X, Data: %02X %02X %02X %02X %02X %02X %02X %02X",
           msg->id, msg->data[0], msg->data[1], msg->data[2], msg->data[3],
           msg->data[4], msg->data[5], msg->data[6], msg->data[7]);
}

int main(int argc, char** argv)
{
  // 初始化ROS节点
  ros::init(argc, argv, "can_motor_node");
  ros::NodeHandle nh;

  const char* ifname = "can0"; // CAN接口名，根据实际情况修改
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
  can_pub = nh.advertise<can_msgs::Frame>("sent_messages", 10);

  // 创建CAN消息订阅者
  ros::Subscriber can_sub = nh.subscribe("received_messages", 10, receiveCallback);

  MC_ProtocolInit();

  // 发布和订阅CAN消息
  while (ros::ok())
  {
    MC_FindOnlineMotor();
    std::cout<< "OnlineMotorNum is:"<<unsigned(MC_GetOnlineMotorNum()) << std::endl;

    // 延时一段时间后再发布下一条CAN消息
    ros::Duration(0.1).sleep();
    
    // 处理订阅的消息
    ros::spinOnce();
  }

  return 0;
}
