#include <iostream>
#include <cstring>
#include <unistd.h>
#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>

int main() {
    const char* ifname = "can0"; // CAN接口名，根据实际情况修改

    // 创建套接字
    int sockfd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
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

    // 准备CAN帧
    struct can_frame frame;
    frame.can_id = 0x123; // CAN标识符，根据实际情况修改
    frame.can_dlc = 8;    // 数据长度，根据实际情况修改
    std::memset(frame.data, 1, sizeof(frame.data)); // 数据初始化为0

    // 发送CAN帧
    if (write(sockfd, &frame, sizeof(frame)) == -1) {
        std::cerr << "Failed to send CAN frame." << std::endl;
        close(sockfd);
        return 1;
    }

    // 关闭套接字
    close(sockfd);

    std::cout << "CAN frame sent successfully." << std::endl;

    return 0;
}
