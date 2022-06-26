#ifndef ICHTHUS_V2X_HPP_
#define ICHTHUS_V2X_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <boost/thread/thread.hpp>
#include <thread>
#include <mutex>

#include <stdio.h>
#include <stdint.h>
#include <time.h>
#include <unistd.h> 
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include "j2735/MessageFrame.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "kiapi_msgs/msg/v2xinfo.hpp"
#include "kiapi_msgs/msg/pvdinfo.hpp"    /* DEBUG */
#include "kiapi_msgs/msg/mylocation.hpp" /* DEBUG */

#define DEBUG_OPTION
 
#define OBU_RECEIVE_BUFFER_SIZE 2048 //받는 buffer의 전체 크기
#define MAX_UPER_SIZE 1400 //buffer 중 UPER(Payload) 크기

#define PVD_INTERVAL 200 //msec(5hz)

/* =============CHANGE IP & PORT============== */
// #define IP "118.45.183.36" // OBU = "192.168.10.10" 테스트서버 - "118.45.183.36", 레코딩서버 - "127.0.0.1"
#define PORT 23000 // 레코딩옛날 - 15130, 레코딩지금 - 23000
/* =========================================== */

using namespace std;


enum J2735_DsrcID
{
    DSRC_ID_MAP = 18,
    DSRC_ID_SPAT = 19,
    DSRC_ID_BSM = 20 
};

struct Vehicle_Location_Information
{
  int32_t latitude;  // 1/10th microdegree (-900000000..900000001)
  int32_t longitude; // 1/10th microdegree (-1799999999..1800000001)
  int32_t elevation; // units of 0.1 m (-4096..61439)
  uint16_t heading;  // 0.0125 degree (0..28800)
  uint16_t speed;    // units of 0.02 m/s (0..8191)
  int transmisson;   // enum (0..7)
};

/* TX_WAVE_UPER HEADER */
struct CestObuUperPacketHeader
{
  unsigned short messageType; // 메시지의 타입
  unsigned char seq;          // 메시지의 시퀀스
  unsigned short payloadLen;  // 페이로드 길이
  unsigned char deviceType;   // 장치 종류
  unsigned char deviceId[3];  // 장치 ID (MAC 하위 3바이트)
}__attribute__((packed));

/* TX_WAVE_UPER_RESULT PAYLOAD */
struct TxWaveUperResultPayload
{
  unsigned char txWaveUperSeq;
  unsigned char resultCode;
  unsigned char size;
}__attribute__((packed)); 


namespace ichthus_v2x
{

class IchthusV2X : public rclcpp::Node
{
public:
  explicit IchthusV2X();
  virtual ~IchthusV2X();

  // void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg); /* DEBUG */
  // void LocationCallback(const kiapi_msgs::msg::Mylocation::SharedPtr msg); /* DEBUG */
  void GnssCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void GnssPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void VelocityCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  void sendPvd();
  void gen_PvdMsg(); /* DEBUG */
  void receiveSpat();
  void connect_obu_uper_tcp();
  void install_signal_handler();

  void gen_SPaTMsg(SPAT_t *spat);
  void print_hex(char *data, int len);
  static void sigint_handler(int sig);
  
  int tx_v2i_pvd();
  int parse_spat(SPAT_t *spat);
  int parse_pvd(MessageFrame_t *pvd);
  int fill_j2735_pvd(MessageFrame_t *dst);
  int encode_j2735_uper(char *dst, MessageFrame_t *src);
  int request_tx_wave_obu(char *uper, unsigned short uperLength);

  unsigned long long get_clock_time();

  std::thread receiveSpatThread();
  std::thread sendPvdThread();

private:
  // rclcpp::Publisher<kiapi_msgs::msg::Pvdinfo>::SharedPtr pvd_pub_;       /* DEBUG */
  // rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr ori_sub_;       /* DEBUG */
  // rclcpp::Subscription<kiapi_msgs::msg::Mylocation>::SharedPtr loc_sub_; /* DEBUG */
  rclcpp::Publisher<kiapi_msgs::msg::V2xinfo>::SharedPtr v2x_pub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr loc_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr vel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr gnss_pose_sub_;

  kiapi_msgs::msg::V2xinfo v2x_msg;
  kiapi_msgs::msg::Pvdinfo pvd_msg; /* DEBUG */

  Vehicle_Location_Information Vli_;

  std::string IP;
  std::thread th1;
  std::thread th2;

  int sockFd, uperSize;
  uint8_t rxBuffer[OBU_RECEIVE_BUFFER_SIZE],rxUperBuffer[MAX_UPER_SIZE];
  unsigned long long txPvd;

  /* Based Information of Vechicle Sysmtem that connect OBU */
  uint8_t packetSeq = 0; //TCP 전송 Header 내 Sequence Number 값 
  unsigned char clientDeviceId[3] = {0x01,0x02,0x03};   // Device ID in TCP Header(send)
  unsigned char temporaryId[4] = {0x00,0x01,0x02,0x03}; // Temporary ID in Msg when send BSM, PVD

};

}
#endif // ICHTHUS_V2X_HPP_