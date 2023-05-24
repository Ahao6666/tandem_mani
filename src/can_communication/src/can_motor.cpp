#include "can_motor.h"
mcProtocol_t mcProtocol;
struct can_frame frame;
int sockfd;

void MC_ProtocolEncode(uint8_t index, uint8_t mode, uint8_t cmd, uint8_t* buf)
{
    // 准备CAN帧
    frame.can_id = index; // CAN标识符，根据实际情况修改
    frame.can_dlc = 8;    // 数据长度，根据实际情况修改
    std::memset(frame.data, 0, sizeof(frame.data)); // 数据初始化为0
	frame.data[0] = mode;
	frame.data[1] = cmd;
	frame.data[2] = buf[0];
	frame.data[3] = buf[1];
	frame.data[4] = buf[2];
	frame.data[5] = buf[3];
	frame.data[6] = buf[4];
	frame.data[7] = buf[5];
    write(sockfd, &frame, sizeof(frame));
}
void MC_ProtocolInit(void)
{
	for(uint8_t i = 0; i<0xFF; i++)
	{
		mcProtocol.connectFlag[i] = 0;
		mcProtocol.connectID[i] = 0;
	}
	mcProtocol.onlineNum = 0;
	mcProtocol.index = 0;
	mcProtocol.initFinish = 1;
}

void MC_FindOnlineMotor(void)
{
	mcProtocol.refreshFlag = 1;
	for(uint8_t i = 0;i<0xFF; i++ )
	{
		MC_HeartbeatEncode(i);
		/* connect clean */
		mcProtocol.connectFlag[i] = 0;
	}
}
void MC_HeartbeatEncode(uint8_t index)
{
	memset(canDataToSend, 0, sizeof(canDataToSend));
	MC_ProtocolEncode(index, MODE_HEARTBEAT , 0x01 , canDataToSend);
}
uint8_t MC_GetOnlineMotorNum(void)
{
	return mcProtocol.onlineNum;
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
void receiveCallback(const struct can_frame* frame)
{
    // 处理接收到的CAN帧数据
    // 在这里你可以根据CAN帧的ID、数据等信息进行相应的处理
    // 例如打印CAN帧的ID和数据
    printf("Received CAN frame - ID: 0x%03X, Data: %02X %02X %02X %02X %02X %02X %02X %02X\n",
           frame->can_id, frame->data[0], frame->data[1], frame->data[2], frame->data[3],
           frame->data[4], frame->data[5], frame->data[6], frame->data[7]);
}
int main(int argc, char **argv) {

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
    //找到在线的电机设备
    MC_FindOnlineMotor();
    // std::cout << "motor_num is:" << MC_GetOnlineMotorNum() << std::endl;
    ssize_t nbytes = read(sockfd, &frame, sizeof(frame));
    if (nbytes < 0) {
        perror("Failed to receive CAN frame");
    }
    receiveCallback(&frame);

    // 关闭套接字
    close(sockfd);

    std::cout << "CAN frame sent successfully." << std::endl;

    return 0;
}
