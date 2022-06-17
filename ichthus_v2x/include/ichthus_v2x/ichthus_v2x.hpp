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
#include <time.h>
#include <unistd.h> 
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include "j2735/MessageFrame.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "kiapi_msgs/msg/v2xinfo.hpp"
#include "kiapi_msgs/msg/pvdinfo.hpp"    // TEST VER.
#include "kiapi_msgs/msg/mylocation.hpp" // TEST VER.

#define DEBUG_OPTION
 
#define OBU_RECEIVE_BUFFER_SIZE 2048 //받는 buffer의 전체 크기
#define MAX_UPER_SIZE 1400 //buffer 중 UPER(Payload) 크기

#define PVD_INTERVAL 200 //msec

/* =============CHANGE IP & PORT============== */
#define IP "127.0.0.1" // OBU = "192.168.10.10 테스트서버 - "118.45.183.36", 레코딩서버 - "127.0.0.1"
#define PORT 23000 // 레코딩옛날 - 15130, 레코딩지금 - 23000
/* =========================================== */

using namespace std;


enum J2735_DsrcID
{
    DSRC_ID_MAP = 18,
    DSRC_ID_SPAT = 19,
    DSRC_ID_BSM = 20 
};

struct Vehicle_Location_Information//100ms 주기로 V2x 서버에게 현재 차량의 위치를 전송해야 함. 
{
  double latitude;            // degree
  double longitude;           // degree
  double elevation;           // cm
  long int heading;          // degree
  int speed;        // km/h
  int transmisson;  // gear
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


//explicit : 자신이 원하지 않은 형변환이 일어나지 않도록 제한하는 키워드

namespace ichthus_v2x
{

class IchthusV2X : public rclcpp::Node //rclcpp의 Node 클래스 상속받는 IchthusV2X 클래스
{
public:
  explicit IchthusV2X();
  virtual ~IchthusV2X(); //소멸자를 가상함수로 선언하지 않으면 자식 클래스의 소멸자는 호출되지 않음 

  // void LocationCallback(const kiapi_msgs::msg::Mylocation::SharedPtr msg);
  void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void GnssCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void VelocityCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  void sendPvd();
  void receiveSpat();
  void connect_obu_uper_tcp();
  void install_signal_handler();

  void gen_SPaTMsg(SPAT_t *spat);
  void gen_PvdMsg(MessageFrame_t *pvd);
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
  rclcpp::Publisher<kiapi_msgs::msg::V2xinfo>::SharedPtr v2x_pub_;            //SPaT 정보 퍼블리시
  // rclcpp::Publisher<kiapi_msgs::msg::Pvdinfo>::SharedPtr pvd_pub_;         //PVD 정보 퍼블리시(test)
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr loc_sub_;      //GNSS 정보 서브스크라이브
  // rclcpp::Subscription<kiapi_msgs::msg::Mylocation>::SharedPtr loc_sub_;   //GNSS 정보 서브스크라이브(test)
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr ori_sub_;            //IMU 정보 서브스크라이브
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr vel_sub_; //IMU 정보 서브스크라이브
  
  kiapi_msgs::msg::V2xinfo v2x_msg; //V2x_info.msg
  kiapi_msgs::msg::Pvdinfo pvd_msg; //Pvd_info.msg

  Vehicle_Location_Information Vli_;

  std::thread th1;
  std::thread th2;
//   std::mutex lock;

  // bool done = false;
  // bool is_first_topic = true;

  int sockFd, uperSize;
  uint8_t rxBuffer[OBU_RECEIVE_BUFFER_SIZE],rxUperBuffer[MAX_UPER_SIZE];
  unsigned long long txPvd;

  // OBU와 연결할 차량시스템의 기본 정보
  uint8_t packetSeq = 0; //TCP 전송 Header 내 Sequence Number 값 
  unsigned char clientDeviceId[3] = {0x01,0x02,0x03}; //TCP 전송 Header 내 Device ID
  unsigned char temporaryId[4] = {0x00,0x01,0x02,0x03}; //BSM, PVD 전송 시 메시지 내 Temporary ID

};

}
#endif // ICHTHUS_V2X_HPP_