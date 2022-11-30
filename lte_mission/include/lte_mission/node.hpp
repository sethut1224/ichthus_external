/*  ===========================================================================
    Copyright 2022. The ICHTHUS Project. All Rights Reserved.
    Sumin In (ism0705@naver.com),
    Youngjun Han (young@ssu.ac.kr)
    Vision System Laboratory, Soongsil University.
    added by ICHTHUS, Sumin In on 20221026
    [Licensed under the MIT License]  
    ===========================================================================*/

#ifndef LTE_MISSION_HPP_
#define LTE_MISSION_HPP_

#include "lte_mission/obu_packet.hpp"

namespace lte
{

class LTE : public rclcpp::Node
{
public:
  explicit LTE();
  virtual ~LTE();

  void SelectedMissionCallback(const std_msgs::msg::UInt8::ConstSharedPtr msg);
  void ArriveGoalCallback(const std_msgs::msg::Bool::ConstSharedPtr msg);
  void ArriveStartingPointCallback(const std_msgs::msg::Bool::ConstSharedPtr msg);

  void ConnectSocket();
  void ReceivePacket();

  int RecvMissionStage1(unsigned char* buf);
  int RecvMissionStage2(unsigned char* buf); 
  int RecvRequestAck(unsigned char* buf); 
  int RecvItemAck(unsigned char* buf);
  int RecvNodeAck(unsigned char* buf);

  void ParseMissionStage1(MissionListStage1 *msg, unsigned char* buf);
  void ParseMissionStage2(MissionListStage2 *msg, unsigned char* buf);
  void ParseRequestAck(Request_Ack *msg, unsigned char* buf);
  void ParseItemAck(Item_Ack *msg, unsigned char* buf);
  void ParseNodeAck(Request_Ack *msg, unsigned char* buf);

  void PrintMissionStage1(MissionListStage1 *msg);
  void PrintMissionStage2(MissionListStage2 *msg);
  void PrintRequestAck(Request_Ack *msg);
  void PrintItemAck(Item_Ack *msg);
  void PrintNodeAck(Request_Ack *msg);

  void PubMissonList1(MissionListStage1 *msg);
  void PubMissonList2(MissionListStage2 *msg);
  
  void SendRequest(unsigned char id, unsigned char req);

  std::thread ReceivePacketThread();

private:
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr selected_mission_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arrive_starting_point_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arrive_goal_sub_;
  rclcpp::Publisher<kiapi_msgs::msg::MissionListStage1>::SharedPtr stage1_mission_pub_;
  rclcpp::Publisher<kiapi_msgs::msg::MissionListStage2>::SharedPtr stage2_mission_pub_;
  rclcpp::Publisher<kiapi_msgs::msg::CurrentItem>::SharedPtr current_item_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr preempted_mission_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr error_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr passed_node_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr arrive_response_pub_;

  rclcpp::Time current_time;

  bool DEBUG;
  bool DISPLAY;
  std::string IP;

  bool arrive_starting_point = false;
  bool arrive_goal = false;
  bool is_selected = false;

  uint8_t seq = 0;
  uint8_t selected_mission;
  unsigned char* temp_buf;
  unsigned char recvbuf[MAX_BUF_LEN] = {0,};

  int clnt_sock;

  std::thread th1;

  int print_count = 0;
};

} 
#endif // LTE_MISSION_HPP_