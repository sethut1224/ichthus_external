/*  ===========================================================================
    Copyright 2022. The ICHTHUS Project. All Rights Reserved.
    Sumin In (ism0705@naver.com),
    Youngjun Han (young@ssu.ac.kr)
    Vision System Laboratory, Soongsil University.
    added by ICHTHUS, Sumin In on 20221026
    [Licensed under the MIT License]  
    ===========================================================================*/
    
#ifndef ICHTHUS_V2X_HPP_
#define ICHTHUS_V2X_HPP_

#include "ichthus_v2x/obu_packet.hpp"


namespace ichthus_v2x
{

class IchthusV2X : public rclcpp::Node
{
public:
  explicit IchthusV2X();
  virtual ~IchthusV2X();

  void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg); /* DEBUG */
  void GnssCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void GnssPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void VelocityCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  void ConnectSocket();

  void ReceiveSpat();
  void PrintSpat(SPAT_t *spat);
  void PublishSpat(SPAT_t *spat);

  void SendPvd();
  void FillPvd(MessageFrame_t *dst);
  void PrintPvd(MessageFrame_t *pvd);
  void PrintHex(char *data, int len);
  void PublishPvd(); /* DEBUG */
  
  int V2iPvd();
  int EncodeJ2735Uper(char *dst, MessageFrame_t *src);
  int SendRequest(char *uper, unsigned short uperLength);

  unsigned long long GetClockTime();

  std::thread ReceiveSpatThread();
  std::thread SendPvdThread();

private:
  rclcpp::Publisher<kiapi_msgs::msg::Pvdinfo>::SharedPtr pvd_pub_;       /* DEBUG */
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr ori_sub_;       /* DEBUG */
  // rclcpp::Publisher<kiapi_msgs::msg::V2xinfo>::SharedPtr v2x_pub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr loc_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr vel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr gnss_pose_sub_;
  rclcpp::Publisher<autoware_auto_perception_msgs::msg::TrafficSignalArray>::SharedPtr v2x_pub_;

  // kiapi_msgs::msg::V2xinfo v2x_msg;

  Vehicle_Location_Information Vli_;

  bool DEBUG;
  bool DISPLAY;
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