/*  ===========================================================================
    Copyright 2022. The ICHTHUS Project. All Rights Reserved.
    Sumin In (ism0705@naver.com),
    Youngjun Han (young@ssu.ac.kr)
    Vision System Laboratory, Soongsil University.
    added by ICHTHUS, Sumin In on 20221026
    [Licensed under the MIT License]  
    ===========================================================================*/

#include "lte_mission/node.hpp"

namespace lte
{

LTE::LTE() : Node("lte_mission")
{
  DEBUG = this->declare_parameter("debug", true);
  DISPLAY =  this->declare_parameter("display", true);
  IP = this->declare_parameter("ip", "192.168.10.10");

  ConnectSocket();

  auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
  selected_mission_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
                          "/v2x/selected_mission", qos_profile,
                          std::bind(&LTE::SelectedMissionCallback, this, std::placeholders::_1));
  arrive_starting_point_sub_ = this->create_subscription<std_msgs::msg::Bool>(
                              "/v2x/starting_point_arrive", qos_profile,
                              std::bind(&LTE::ArriveStartingPointCallback, this, std::placeholders::_1));
  arrive_goal_sub_ = this->create_subscription<std_msgs::msg::Bool>(
                    "/v2x/goal_arrive", qos_profile,
                    std::bind(&LTE::ArriveGoalCallback, this, std::placeholders::_1));
  stage1_mission_pub_ = this->create_publisher<kiapi_msgs::msg::MissionListStage1>(
                        "/v2x/stage1_mission_list", qos_profile);
  stage2_mission_pub_ = this->create_publisher<kiapi_msgs::msg::MissionListStage2>(
                        "/v2x/stage2_mission_list", qos_profile);
  current_item_pub_ = this->create_publisher<kiapi_msgs::msg::CurrentItem>("/v2x/current_item", qos_profile);
  preempted_mission_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/v2x/preempted_mission", qos_profile);
  error_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/v2x/error", qos_profile);
  passed_node_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/v2x/passed_node", qos_profile); // 차량 제어 사용 권장 x
  arrive_response_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/v2x/arrive_response", qos_profile);
}
LTE::~LTE()
{
}

void LTE::SelectedMissionCallback(const std_msgs::msg::UInt8::ConstSharedPtr msg)
{
  std::cout << "SelectedMissionCallback!" << std::endl;
  selected_mission = msg->data;
}

void LTE::ArriveStartingPointCallback(const std_msgs::msg::Bool::ConstSharedPtr msg)
{
  std::cout << "ArriveStartingPointCallback!!" << std::endl;
  arrive_starting_point = msg->data;
}

void LTE::ArriveGoalCallback(const std_msgs::msg::Bool::ConstSharedPtr msg)
{
  std::cout << "ArriveGoalCallback!!!" << std::endl;
  arrive_goal = msg->data;
}

std::thread LTE::ReceivePacketThread()
{
  return std::thread([this]{ReceivePacket();});
}

void LTE::ConnectSocket()
{
  clnt_sock = -1;
  std::string serv_ip = IP;
  uint32_t serv_port = PORT;

  struct sockaddr_in serv_addr;
  memset(&serv_addr, 0, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_port = htons(serv_port);

  if (inet_pton(AF_INET, serv_ip.c_str(), &serv_addr.sin_addr) <= 0)
  {
    std::cout << "ERROR[ConnectSocket] : Error, inet_pton" << std::endl;
    exit(1);
  }

  if ((clnt_sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0)
  {
    std::cout << "ERROR[ConnectSocket] : connect failed, retry" << std::endl;
    exit(1);
  }

  int opt_val = 1;
  setsockopt(clnt_sock, IPPROTO_TCP, TCP_NODELAY, (void*)&opt_val, sizeof(opt_val));
  setsockopt(clnt_sock, SOL_SOCKET, SO_REUSEADDR, (void*)&opt_val, sizeof(opt_val));
  
  int connected = -1;
  if ((connected = connect(clnt_sock, (const sockaddr *)&serv_addr, sizeof(serv_addr))) < 0)
  {
    std::cout << "ERROR[ConnectSocket] : step sock connect error : " << connected << std::endl;
    exit(1);
  }

  std::cout << "OBU TCP [" << serv_ip << ":" << serv_port << "] Connected" << std::endl;

  th1 = ReceivePacketThread();
}

void LTE::ReceivePacket()
{
  std::string buf;
  int pktsize = -1;
  while ((pktsize = recv(clnt_sock, recvbuf, sizeof(recvbuf), 0)) > 0)
  {
    print_count++;

    if (pktsize <= MAX_BUF_LEN)
    {
      if (buf.size() > MAX_BUFFER_LEN)
      {
        buf.clear();
      }
      buf.append((char*)recvbuf, pktsize);
    }

    if (buf.size() < sizeof(struct MsgHeader))
    {
      std::cout <<  "ERROR[ReceivePacket] : require more bytes, current bytes = " << pktsize << std::endl;
      break;
    }

    temp_buf = (unsigned char*)buf.c_str();
    struct MsgHeader *header = (struct MsgHeader*)temp_buf;
    if (buf.size() < sizeof(struct MsgHeader)+header->payload_length)
    {
      std::cout << "ERROR[ReceivePacket] : pay size error : " << pktsize-sizeof(struct MsgHeader) << "(recv) < " 
                << header->payload_length << "(value)" << std::endl;
      break;
    }

    // printf("\nmessage_type : 0x%04X\n", header->message_type);
    switch (header->message_type)
    {
      case MessageType::MISSION_LIST_STAGE1 : // 341A        
        RecvMissionStage1(temp_buf);
        break;
      case MessageType::MISSION_LIST_STAGE2 : // 341B
        RecvMissionStage2(temp_buf);
        break;
      case MessageType::AUTONOMOUS_RESPONSE : // 341C
        RecvRequestAck(temp_buf);
        break;
      case MessageType::MISSION_STAGE_2_ITEM_GET : // 341D
        RecvItemAck(temp_buf);
        break;
      case MessageType::MISSION_STAGE_1_STATE_NODE : // 341E
        RecvNodeAck(temp_buf);
        break;
      default:
        std::cout << "[ReceivePacket] recv unkown message" << std::endl;
        buf.clear();
        break;
    }
    // buf.erase(0, sizeof(struct MsgHeader) + header->payload_length);
    buf.clear();
    usleep(1000);
  }
  shutdown(clnt_sock, SHUT_RDWR);
  close(clnt_sock);
}


int LTE::RecvMissionStage1(unsigned char* buf)
{
  std::cout << "\n========================" << std::endl;
  if (DEBUG == true)
  {
    std::cout << "[RecvMissionStage1] Start" << std::endl;
  }

  struct MissionListStage1 msg = {};
  ParseMissionStage1(&msg, buf);
  
  if (print_count%5 == 0)
  {
    PrintMissionStage1(&msg);
  }

  PubMissonList1(&msg);
  std::cout << "\n[RecvMissionStage1] Publish Mission Stage1 List" << std::endl;
  
  if (msg.mission_count > 0)
  {
    for (int i=0; i<msg.mission_count; i++)
    {
      if (selected_mission == msg.mission_list[i].mission_id)
      {
        std::cout << "[RecvMissionStage1] select mission => ";
        if (selected_mission == 1)
        {
          std::cout << "Easy(1)" << std::endl;
        }
        else if (selected_mission == 2)
        {
          std::cout << "Normal(2)" << std::endl;
        }
        else if (selected_mission == 3)
        {
          std::cout << "Hard(3)" << std::endl;
        }

        /* 스테이지1 시작 또는 테스트 모드 */
        if (msg.mission_status==1 || msg.mission_status==4)
        {
          /* 선택 대기 및 미션 선택 요청 x*/
          if (msg.mission_list[i].status == 0x00 && is_selected == false)
          {
            std::cout << "[RecvMissionStage1] Request Select Mission" << std::endl;
            sleep(1);
            SendRequest(selected_mission, RequestType::REQ_SELECT_MISSION);
          }
          /* 선택 완료 또는 미션 선택 요청 o 및 출발지 도착 */
          else if ((msg.mission_list[i].status == 0x01 || is_selected == true) && arrive_starting_point == true)
          {
            std::cout << "[RecvMissionStage1] Request Arrive Start Point" << std::endl;
            sleep(1);
            SendRequest(selected_mission, RequestType::REQ_START_POSITION);
          }
          /* 선택 완료 또는 미션 선택 요청 o 및 도착지 도착*/
          else if ((msg.mission_list[i].status == 0x01 || is_selected == true) && arrive_goal == true)
          {
            std::cout << "[RecvMissionStage1] Request Arrive Goal" << std::endl;
            sleep(1);
            SendRequest(selected_mission, RequestType::REQ_END_POSITION);
          }
        }
        else if (msg.mission_status == 3)
        {
          std::cout << "[RecvMissionStage1] Mission Over" << std::endl;
          shutdown(clnt_sock, SHUT_RDWR);
          close(clnt_sock);
          exit(1);
        }
      }
    }
    std::cout << "========================" << std::endl;
  }

  if (msg.mission_list != nullptr)
  {
    delete msg.mission_list;
  }
  if (msg.mission_route_list != nullptr)
  {
    delete msg.mission_route_list;
  }

  return 0;
}

int LTE::RecvMissionStage2(unsigned char* buf)
{
  std::cout << "\n========================" << std::endl;
  if (DEBUG == true)
  {
    std::cout << "[RecvMissionStage2] Start" << std::endl;
  }

  struct MissionListStage2 msg = {};
  ParseMissionStage2(&msg, buf);

  if (DISPLAY == true && print_count%5 == 0)
  {
    PrintMissionStage2(&msg);
  }

  PubMissonList2(&msg);
  std::cout << "\n[RecvMissionStage2] Publish Mission Stage2 List" << std::endl;

  if(msg.item_list != nullptr)
  {
    delete msg.item_list;
  }

  return 0;
}

int LTE::RecvRequestAck(unsigned char* buf)
{
  std::cout << "\n========================" << std::endl;
  if (DEBUG == true)
  {
    std::cout << "[RecvRequestAck] Start" << std::endl;
  }

  struct Request_Ack msg = {};
  ParseRequestAck(&msg, buf);

  if (DISPLAY == true && print_count%5 == 0)
  {
    PrintRequestAck(&msg);
  }

  /* 미션 수락 요청 (성공) */
  if (msg.response==ResponseType::RES_SUCCESS && msg.request==RequestType::REQ_SELECT_MISSION)
  {
    std::cout << "[RecvRequestAck] : Mission selection complete!" << std::endl;
    is_selected = true;
    std_msgs::msg::UInt8 preempted_mission;
    preempted_mission.data = selected_mission;
    preempted_mission_pub_->publish(preempted_mission);
    std::cout << "[RecvRequestAck] : Publish Selected Mission to Mission Manager!!" << std::endl;
  }
  /* 출발지 도착 완료 요청 (성공) */
  else if (msg.response==ResponseType::RES_SUCCESS && msg.request==RequestType::REQ_START_POSITION)
  {
    std::cout << "[RecvRequestAck] : Arrived at the start point" << std::endl;
    arrive_starting_point = false;
    std_msgs::msg::UInt8 arrive_response;
    arrive_response.data = 1;
    arrive_response_pub_->publish(arrive_response);
    std::cout << "[RecvRequestAck] : Publish Arrived Start Point to Mission Manager!!" << std::endl;
  }
  /* 도착지 도착 완료 요청 (성공) */
  else if (msg.response == ResponseType::RES_SUCCESS && msg.request == RequestType::REQ_END_POSITION)
  {
    std::cout << "[RecvRequestAck] : Arrived at the destination" << std::endl;
    arrive_goal = false;
    std_msgs::msg::UInt8 arrive_response;
    arrive_response.data = 2;
    arrive_response_pub_->publish(arrive_response);
    std::cout << "[RecvRequestAck] : Publish Arrived Goal to Mission Manager!!" << std::endl;
  }
  /* V2X 서버와 통신이 끊어진 상태 */
  else if (msg.response == 0x01)
  {
    std::cout << "ERROR[RecvRequestAck] : Server disconnected. Try again" << std::endl;
    std::cout << "========================" << std::endl;
    exit(1);
    return 0;
  }
  /* 대회 시작 상태 아님 */
  else if (msg.response == 0x02)
  {
    std::cout << "ERROR[RecvRequestAck] : It's not mission start" << std::endl;
    std_msgs::msg::UInt8 error;
    error.data = 0x02;
    error_pub_->publish(error);
    if (DEBUG == true)
    {
      std::cout << "[RecvRequestAck] : Publish Error Code!!" << std::endl;
    }
    std::cout << "========================" << std::endl;
    return 0;
  }
  /* 이미 선택된 미션이 존재함에도 다른 미션 ID의 선택 요청을 보낸 경우 */
  else if (msg.response == 0x03)
  {
    std::cout << "ERROR[RecvRequestAck] : Already have mission" << std::endl;
    std_msgs::msg::UInt8 error;
    error.data = 0x03;
    error_pub_->publish(error);
    if (DEBUG == true)
    {
      std::cout << "[RecvRequestAck] : Publish Error Code!!" << std::endl;
    }
    std::cout << "========================" << std::endl;
    return 0;
  }
  /* 미션 판정 중 (아직 10초가 정차되지 않았거나, 평가자가 아직 판정이 완료되지 않은 상태)*/
  /* 자율주행차량은 성공 또는 실패 응답이 수신되기까지 10초가 지났어도 자율주행으로 출발하지 않도록 주의 */
  else if (msg.response == 0x04)
  {
    std::cout << "[RecvRequestAck] : Judging the mission" << std::endl;
    std_msgs::msg::UInt8 error;
    std_msgs::msg::UInt8 arrive_response;
    error.data = 0x04;
    arrive_response.data = 0;
    error_pub_->publish(error);
    arrive_response_pub_->publish(arrive_response);
    if (DEBUG == true)
    {
      std::cout << "[RecvRequestAck] : Publish Error Code!!" << std::endl;
    }
    std::cout << "========================" << std::endl;
    return 0;
  }
  /* Request 메시지 내 Request 필드 값이 정의되지 않은 값을 전송한 경우 */
  else if (msg.response == 0x10)
  {
    std::cout << "ERROR[RecvRequestAck] : No request field value defined in Request msg" << std::endl;
    std::cout << "========================" << std::endl;
    exit(1);
    return 0;
  }
  /* Request 메시지 내 Response 필드 값이 0x00 이 아닌 값을 전송한 경우 */
  else if (msg.response == 0x11)
  {
    std::cout << "ERROR[RecvRequestAck] : No 0x00 response filed value in Request msg" << std::endl;
    std::cout << "========================" << std::endl;
    exit(1);
    return 0;
  }
  /* Request 메시지 내 Mission List에서 존재하지 않는 Mission ID를 요청한 경우 */
  else if (msg.response == 0x12)
  {
    std::cout << "ERROR[RecvRequestAck] : Mission ID doesn't exit in the Misison List in Request msg" << std::endl;
    std_msgs::msg::UInt8 error;
    error.data = 0x12;
    error_pub_->publish(error);
    if (DEBUG == true)
    {
      std::cout << "[RecvRequestAck] : Publish Error Code!!" << std::endl;
    }
    std::cout << "========================" << std::endl;
    return 0;
  }
  /* 출발지, 목적지 도착 완료 요청 시 Mission ID가 수락된 Mission ID와 일치하지 않는 경우 */
  else if (msg.response == 0x13)
  {
    std::cout << "ERROR[RecvRequestAck] : Mission ID doesn't match accepted Mission ID when requesting start and destination arrival" << std::endl;
    std_msgs::msg::UInt8 error;
    std_msgs::msg::UInt8 arrive_response;
    error.data = 0x13;
    arrive_response.data = 0;
    error_pub_->publish(error);
    arrive_response_pub_->publish(arrive_response);
    if (DEBUG == true)
    {
      std::cout << "[RecvRequestAck] : Publish Error Code!!" << std::endl;
    }
    std::cout << "========================" << std::endl;
    return 0;
  }
  /* Request 메시지 내 Temporary 필드 값이 모두 0x00이 아닌 메시지를 전송한 경우 */
  else if (msg.response == 0x14)
  {
    std::cout << "ERROR[RecvRequestAck] : No all 0x00 in Temporary field value in Request msg" << std::endl;
    std::cout << "========================" << std::endl;
    exit(1);
    return 0;
  }
  /* End Point 값이 0x0D0A가 아닌 경우 */
  else if (msg.response == 0x15)
  {
    std::cout << "ERROR[RecvRequestAck] : End Point is not 0x0D0A" << std::endl;
    std::cout << "========================" << std::endl;
    exit(1);
    return 0;
  }
  /* Request 메시지 간격이 1sec 이내로 요청한 경우 */
  else if (msg.response == 0x16)
  {
    std::cout << "ERROR[RecvRequestAck] : Request msg interval within 1 sec" << std::endl;
    std::cout << "========================" << std::endl;
    return 0;
  }
  /* Mission List가 수신되지 않은 상태에서 Mission 수락 요청을 전송한 경우 */
  else if (msg.response == 0x17)
  {
    std::cout << "ERROR[RecvRequestAck] : Receive Mission List first" << std::endl;
    std_msgs::msg::UInt8 error;
    error.data = 0x17;
    error_pub_->publish(error);
    if (DEBUG == true)
    {
      std::cout << "[RecvRequestAck] : Publish Error Code!!" << std::endl;
    }
    std::cout << "========================" << std::endl;
    return 0;
  }
  /* Mission ID가 0x00으로 요청된 경우 */
  else if (msg.response == 0x18)
  {
    std::cout << "ERROR[RecvRequestAck] : Request Mission ID as 0x00. Try again" << std::endl;
    std_msgs::msg::UInt8 error;
    error.data = 0x18;
    error_pub_->publish(error);
    if (DEBUG == true)
    {
      std::cout << "[RecvRequestAck] : Publish Error Code!!" << std::endl;
    }
    std::cout << "========================" << std::endl;
    return 0;
  }
  /* 정차(속도 1Km/h 이내)가 아닌 주행상태에서 출발지, 목적지 도착 완료 요청 메시지를 전송한 경우 */
  else if (msg.response == 0x20)
  {
    std::cout << "ERROR[RecvRequestAck] : Not Stopped" << std::endl;
    std_msgs::msg::UInt8 error;
    std_msgs::msg::UInt8 arrive_response;
    error.data = 0x20;
    arrive_response.data = 0;
    error_pub_->publish(error);
    arrive_response_pub_->publish(arrive_response);
    if (DEBUG == true)
    {
      std::cout << "[RecvRequestAck] : Publish Error Code!!" << std::endl;
    }
    std::cout << "========================" << std::endl;
    return 0;
  }
  /* 출발지, 목적지 위치가 올바르지 않은 장소에서 완료 요청 메시지를 전송한 경우 */
  else if (msg.response == 0x21)
  {
    std::cout << "ERROR[RecvRequestAck] : Starting point, destination has not arrived" << std::endl;
    std_msgs::msg::UInt8 error;
    std_msgs::msg::UInt8 arrive_response;
    error.data = 0x21;
    arrive_response.data = 0;
    error_pub_->publish(error);
    arrive_response_pub_->publish(arrive_response);
    if (DEBUG == true)
    {
      std::cout << "[RecvRequestAck] : Publish Error Code!!" << std::endl;
    }
    std::cout << "========================" << std::endl;
    return 0;
  }
  /* 해당 미션에 대해 출발지 도착 완료가 되지 않은 상태에서 도착지 도착 요청을 한 경우 */
  else if (msg.response == 0x22)
  {
    std::cout << "ERROR[RecvRequestAck] : Arrived at starting point not yet completed" << std::endl;
    std_msgs::msg::UInt8 error;
    std_msgs::msg::UInt8 arrive_response;
    error.data = 0x22;
    arrive_response.data = 0;
    error_pub_->publish(error);
    arrive_response_pub_->publish(arrive_response);
    if (DEBUG == true)
    {
      std::cout << "[RecvRequestAck] : Publish Error Code!!" << std::endl;
    }
    std::cout << "========================" << std::endl;
    return 0;
  }

  std::cout << "========================" << std::endl;
  return 0;
}

int LTE::RecvItemAck(unsigned char* buf)
{
  std::cout << "\n========================" << std::endl;
  if (DEBUG == true)
  {
    std::cout << "[RecvItemAck] Start" << std::endl;
  }

  struct Item_Ack msg = {};
  ParseItemAck(&msg, buf);

  if (print_count%5 == 0)
  {
    PrintItemAck(&msg);
  }

  if(msg.request == RequestType::GET_ITEM)
  {
    if(msg.response == ItemType::ITEM_LAP_TIME_SUB)
    {
      printf("[RecvItemAck] %d Item : Reduce Lap Time\n", msg.item_id);
    }
    if(msg.response == ItemType::ITEM_LAP_TIME_ADD)
    {
      printf("[RecvItemAck] %d Item : Increased Lap Time\n", msg.item_id);
    }
    if(msg.response == ItemType::ITEM_BOOST)
    {
      printf("[RecvItemAck] %d Item : Boost\n", msg.item_id);
    }

    kiapi_msgs::msg::CurrentItem item_msg;
    item_msg.item_id = msg.item_id;
    item_msg.item_type = msg.response;
    current_item_pub_->publish(item_msg);
    std::cout << "\n[RecvItemAck] : Publish Get Item!!" << std::endl;
  }
  std::cout << "========================" << std::endl;
  return 0;
}

int LTE::RecvNodeAck(unsigned char* buf)
{
  std::cout << "\n[RecvNodeAck] Start" << std::endl;

  struct Request_Ack msg = {};
  ParseNodeAck(&msg, buf);

  if (DISPLAY == true && print_count%5 == 0)
  {
    PrintNodeAck(&msg);
  }

  if (msg.request==RequestType::PASS_NODE)
  {
    if (msg.response == 0)
    {
      printf("[RecvNodeAck] : No Pass Node\n");
    }
    else
    {
      printf("[RecvNodeAck] : Pass %d Node\n", msg.response);
    }
    
    // 0은 통과한 Node가 없다는 의미
    std_msgs::msg::UInt8 passed_node;
    passed_node.data = msg.response;
    passed_node_pub_->publish(passed_node);
  }

  return 0;
} 


void LTE::ParseMissionStage1(MissionListStage1 *msg, unsigned char* buf)
{
  if (msg == nullptr)
  {
    std::cout << "[ParseMissionStage1] fail : msg == nullptr" << std::endl;
    return;
  }
  std::cout << "[ParseMissionStage1] Start" << std::endl;

  size_t copy_len_1 = sizeof(MissionListStage1) - sizeof(MissionListStage1::mission_list) - sizeof(MissionListStage1::mission_route_list);
  memmove(msg, buf, copy_len_1);

  size_t copy_len_2 = sizeof(MissionData)*msg->mission_count;
  msg->mission_list = new MissionData[msg->mission_count];
  memmove(msg->mission_list, &buf[copy_len_1], copy_len_2);

  size_t copy_len_3 = sizeof(MissionRouteData)*msg->mission_route_count;
  msg->mission_route_list = new MissionRouteData[msg->mission_route_count];
  memmove(msg->mission_route_list, &buf[copy_len_1+copy_len_2], copy_len_3);
}

void LTE::ParseMissionStage2(MissionListStage2 *msg, unsigned char* buf)
{
  if (msg == nullptr)
  {
    std::cout << "[ParseMissionStage2] fail : msg == nullptr" << std::endl;
    return;
  }
  std::cout << "\n[ParseMissionStage2] Start" << std::endl;

  size_t copy_len_1 = sizeof(MissionListStage2) - sizeof(MissionListStage2::item_list);
  memmove(msg, buf, copy_len_1);

  size_t copy_len_2 = sizeof(ItemData) * msg->item_count;
  msg->item_list = new ItemData[msg->item_count];
  memmove(msg->item_list, &buf[copy_len_1],copy_len_2);
}

void LTE::ParseRequestAck(Request_Ack *msg, unsigned char* buf)
{
  if(msg == nullptr)
  {
    std::cout << "[ParseRequestAck] fail : msg == nullptr" << std::endl;
    return;
  }
  std::cout << "\n[ParseRequestAck] Start" << std::endl;

  memmove(msg, buf, sizeof(Request_Ack)); 
}

void LTE::ParseItemAck(Item_Ack *msg, unsigned char* buf)
{
  if(msg == nullptr)
  {
    std::cout << "[ParseItemAck] fail : msg == nullptr" << std::endl;
    return;
  }
  std::cout << "\n[ParseItemAck] Start" << std::endl;

  memmove(msg, buf, sizeof(Item_Ack));
}

void LTE::ParseNodeAck(Request_Ack *msg, unsigned char* buf)
{
  if(msg == nullptr)
  {
    std::cout << "[ParseNodeAck] fail : msg == nullptr" << std::endl;
    return;
  }
  std::cout << "\n[ParseNodeAck] Start" << std::endl;

  memmove(msg, buf, sizeof(Request_Ack)); 
}


void LTE::PubMissonList1(MissionListStage1 *msg)
{
  kiapi_msgs::msg::MissionListStage1 stage1_msg;

  current_time = this->now();
  stage1_msg.stamp = current_time;

  stage1_msg.mission_status = msg->mission_status;
  stage1_msg.mission_count = msg->mission_count;
  stage1_msg.mission_route_count = msg->mission_route_count;

  for (int i=0; i<msg->mission_count; i++)
  {
    kiapi_msgs::msg::MissionData data;
    data.mission_id = msg->mission_list[i].mission_id;
    data.status = msg->mission_list[i].status;
    data.event_count = msg->mission_list[i].event_count;
    data.distance = msg->mission_list[i].distance;
    data.route_id = msg->mission_list[i].route_id;
    data.start_lat = msg->mission_list[i].start_lat;
    data.start_lon = msg->mission_list[i].start_Lon;
    data.end_lat = msg->mission_list[i].end_lat;
    data.end_lon = msg->mission_list[i].end_lon;
  
    stage1_msg.mission_list.emplace_back(data);
  }

  for (int i=0; i<msg->mission_route_count; i++)
  {
    kiapi_msgs::msg::MissionRouteData route;
    route.mission_route_id = msg->mission_route_list[i].mission_route_id;
    route.route_node_total_count = msg->mission_route_list[i].route_node_total_count;
    route.route_node_index = msg->mission_route_list[i].route_node_index;
    route.route_node_type = msg->mission_route_list[i].route_node_type;
    route.route_node_pos_lat = msg->mission_route_list[i].route_node_pos_lat;
    route.route_node_pos_lon = msg->mission_route_list[i].route_node_pos_lon;

    stage1_msg.mission_route_list.emplace_back(route);
  }
  stage1_mission_pub_->publish(stage1_msg);
}

void LTE::PubMissonList2(MissionListStage2 *msg)
{
  kiapi_msgs::msg::MissionListStage2 stage2_msg;

  current_time = this->now();
  stage2_msg.stamp = current_time;

  stage2_msg.mission_status = msg->mission_status;
  stage2_msg.item_count = msg->item_count;

  for (int i=0; i<msg->item_count; i++)
  {
    kiapi_msgs::msg::ItemData data;
    data.item_id = msg->item_list[i].item_id;
    data.item_type = msg->item_list[i].item_type;
    data.item_status = msg->item_list[i].item_status;
    data.score = msg->item_list[i].score;
    data.speed = msg->item_list[i].speed;
    data.duration = msg->item_list[i].duration;
    data.pos_lat = msg->item_list[i].pos_lat;
    data.pos_long = msg->item_list[i].pos_long;
    data.extend = msg->item_list[i].extend;
    
    stage2_msg.item_list.emplace_back(data);
  }

  stage2_mission_pub_->publish(stage2_msg);
}


void LTE::PrintMissionStage1(MissionListStage1 *msg)
{
  if (msg == nullptr)
  {
    std::cout << "[PrintMissionStage1] fail : msg == nullptr" << std::endl;
    return;
  }

  if (DEBUG == true)
  {
    std::cout << "\n----------------------------" << std::endl;
    std::cout << "[PrintMissionStage1] Start" << std::endl;
    std::cout << "----------------------------" << std::endl;
  }

  printf("mission_status : ");
  switch (msg->mission_status)
  {
    case 0:
      printf("Stand By ");
      break;
    case 1:
      printf("Start ");
      break;
    case 2:
      printf("Abort ");
      break;
    case 3:
      printf("End ");
      break;
    case 4:
      printf("Test ");
      break;
  }
  printf("[%d]\n", msg->mission_status);
  std::cout << "----------------------------" << std::endl;
  
  if (DISPLAY == true)
  {
    printf("mission_list  : %d missions\n", msg->mission_count);
    for (int i=0; i<msg->mission_count; i++)
    {
      printf("\tmission[%d]\n",i);
      printf("\tmission_id  : %d\n", msg->mission_list[i].mission_id);
      printf("\tstatus      : %d\n", msg->mission_list[i].status);
      printf("\tevent_count : %d\n", msg->mission_list[i].event_count);
      printf("\tdistance    : %d\n", msg->mission_list[i].distance);
      printf("\troute_id    : %d\n", msg->mission_list[i].route_id);
      printf("\tstart_lat   : %d\n", msg->mission_list[i].start_lat);
      printf("\tstart_Lon   : %d\n", msg->mission_list[i].start_Lon);
      printf("\tend_lat     : %d\n", msg->mission_list[i].end_lat);
      printf("\tend_lon     : %d\n", msg->mission_list[i].end_lon);
    }
    std::cout << "----------------------------" << std::endl;
  }

  printf("Selected Mission Information\n");
  for (int i=0; i<msg->mission_count; i++)
  {
    if (selected_mission == msg->mission_list[i].mission_id)
    {
      printf("\tmission[%d] ",i);
      if (selected_mission == 1)
      {
        printf("Easy(1)\n");
      }
      else if (selected_mission == 2)
      {
        printf("Normal(2)\n");
      }
      else if (selected_mission == 3)
      {
        printf("Hard(3)");
      }

      printf("\tstatus      : ");
      if (msg->mission_list[i].status == 0)
      {
        printf("Not Selected(0)\n");
      }
      else if (msg->mission_list[i].status == 1)
      {
        printf("Selected(1)");
      }
      // printf("\tevent_count : %d\n", msg->mission_list[i].event_count);
      // printf("\troute_id    : %d\n", msg->mission_list[i].route_id);
      // printf("\tstart_lat   : %d\n", msg->mission_list[i].start_lat);
      // printf("\tstart_Lon   : %d\n", msg->mission_list[i].start_Lon);
      // printf("\tend_lat     : %d\n", msg->mission_list[i].end_lat);
      // printf("\tend_lon     : %d\n", msg->mission_list[i].end_lon);
    }
    std::cout << "----------------------------" << std::endl;
  }

  if (DISPLAY == true)
  {
    printf("mission_route_list       : %d routes\n", msg->mission_route_count);
    for (int i=0; i<msg->mission_route_count; i++)
    {
      printf("\troute[%d]\n",i);
      printf("\tmission_route_id       : %d\n", msg->mission_route_list[i].mission_route_id);
      printf("\troute_node_total_count : %d\n", msg->mission_route_list[i].route_node_total_count);
      printf("\troute_node_index       : %d\n", msg->mission_route_list[i].route_node_index);
      printf("\troute_node_type        : %d\n", msg->mission_route_list[i].route_node_type);
      printf("\troute_node_pos_lat     : %d\n", msg->mission_route_list[i].route_node_pos_lat);
      printf("\troute_node_pos_lon     : %d\n", msg->mission_route_list[i].route_node_pos_lon);
    }
    std::cout << "----------------------------" << std::endl;
  }
}

void LTE::PrintMissionStage2(MissionListStage2 *msg)
{
  if (msg == nullptr)
  {
    std::cout << "[PrintMissionStage2] fail : msg == nullptr" << std::endl;
    return;
  }
  std::cout << "----------------------------" << std::endl;
  std::cout << "[PrintMissionStage2] Start" << std::endl;
  std::cout << "----------------------------" << std::endl;

  printf("mission_status : ");
  switch (msg->mission_status)
  {
    case 0:
      printf("Stand By ");
      break;
    case 1:
      printf("Start ");
      break;
    case 2:
      printf("Abort ");
      break;
    case 3:
      printf("End ");
      break;
    case 4:
      printf("Test ");
      break;
  }
  printf("[%d]\n", msg->mission_status);
  std::cout << "----------------------------" << std::endl;
  
  printf("item_count  : %d items\n", msg->item_count);
  for (int i=0; i<msg->item_count; i++)
  {
    printf("\titem[%d]\n",i);
    printf("\titem_id     : %d\n", msg->item_list[i].item_id);
    
    printf("\titem_type   : ");
    switch (msg->item_list[i].item_type)
    {
      case 1:
        printf("Plus Points");
        break;
      case 2:
        printf("Minus Points");
        break;
      case 3:
        printf("Boost");
        break;
    }
    printf("[%d]\n", msg->item_list[i].item_type);
    
    printf("\titem_status : ");
    switch (msg->item_list[i].item_status)
    {
      case 1:
        printf("Unacquired");
        break;
      case 2:
        printf("Acquired");
        break;
    }
    printf("[%d]\n", msg->item_list[i].item_status);
    
    printf("\tscore       : ");
    if (msg->item_list[i].score == 0)
    {
      printf("Unavailable");
    }
    else if (msg->item_list[i].score < 0)
    {
      printf("Minus %d points", msg->item_list[i].score);
    }
    else if (msg->item_list[i].score > 0)
    {
      printf("Plus %d points", msg->item_list[i].score);
    }
    printf("[%d]\n", msg->item_list[i].score);
    
    printf("\tspeed       : %d km/h\n", msg->item_list[i].speed);
    
    printf("\tduration    : ");
    if (msg->item_list[i].duration == 0)
    {
      printf("Time out"); //지속시간 만료
    }
    else if (msg->item_list[i].duration == 254)
    {
      printf("Max Speed"); //습득 시점부터 완주까지 최대 속도로 주행 가능
    }
    else if (msg->item_list[i].duration == 255)
    {
      printf("Unavailable");
    }
    else
    {
      printf("Hold Time ");
    }
    printf("[%d]\n", msg->item_list[i].duration);
    
    printf("\tpos_lat     : %d\n", msg->item_list[i].pos_lat);
    printf("\tpos_long    : %d\n", msg->item_list[i].pos_long);
    printf("\textend      : %d\n", msg->item_list[i].extend);
  }
  std::cout << "----------------------------" << std::endl;
}

void LTE::PrintRequestAck(Request_Ack *msg)
{
  if(msg == nullptr)
  {
    std::cout << "[PrintRequestAck] fail : msg == nullptr" << std::endl;
    return;
  }
  std::cout << "----------------------------" << std::endl;
  std::cout << "[PrintRequestAck] Start" << std::endl;
  std::cout << "----------------------------" << std::endl;

  printf("mission_id  : %d\n", msg->mission_id);
  printf("reqeust     : %d\n", msg->request);
  printf("response    : %d\n", msg->response);

  printf("description : ");
  for (int i=0; i<(int)sizeof(msg->description); i++)
  {
      printf("%c", msg->description[i]);
  }
  printf("\ntemporary   : %s\n", msg->temporary);
  printf("end_point   : 0x%04X\n", msg->end_point);
}

void LTE::PrintNodeAck(Request_Ack *msg)
{
  if(msg == nullptr)
  {
    std::cout << "[PrintNodeAck] fail : msg == nullptr" << std::endl;
    return;
  }
  std::cout << "----------------------------" << std::endl;
  std::cout << "[PrintNodeAck] Start" << std::endl;
  std::cout << "----------------------------" << std::endl;

  printf("mission_id  : %d\n", msg->mission_id);
  printf("reqeust     : %d\n", msg->request);
  printf("response    : No.%d Pass\n", msg->response);

  printf("description : ");
  for (int i=0; i<(int)sizeof(msg->description); i++)
  {
      printf("%c", msg->description[i]);
  }

  printf("\ntemporary   : %s\n", msg->temporary);
  printf("end_point   : 0x%04X\n\n", msg->end_point);
}


void LTE::PrintItemAck(Item_Ack *msg)
{
  if(msg == nullptr)
  {
    std::cout << "[PrintItemAck] fail : msg == nullptr" << std::endl;
    return;
  }

  if (DEBUG == true)
  {
    std::cout << "\n----------------------------" << std::endl;
    std::cout << "[PrintItemAck] Start" << std::endl;
    std::cout << "----------------------------" << std::endl;
  }

  if (DISPLAY == true)
  {
    printf("item_id     : %d\n", msg->item_id);
    printf("request     : %d\n", msg->request);
    printf("response    : %d", msg->response);
    if (msg->response == 0x01)
    {
      printf("(minus)\n");
    }
    else if (msg->response == 0x02)
    {
      printf("(plus)\n");
    }
    else if (msg->response == 0x03)
    {
      printf("(boost)\n");
    }

    printf("description : %s\n", msg->description);
    printf("temporary   : %s\n", msg->temporary);
    printf("end_point   : 0x%04X\n", msg->end_point);
    printf("\n");
  }
  
  printf("item_id     : %d\n", msg->item_id);
  printf("response    : %d", msg->response);
  if (msg->response == 0x01)
  {
    printf("(minus)\n");
  }
  else if (msg->response == 0x02)
  {
    printf("(plus)\n");
  }
  else if (msg->response == 0x03)
  {
    printf("(boost)\n");
  }
}


void LTE::SendRequest(unsigned char id, unsigned char req)
{
  struct Request msg = {0,};
  msg.header.message_type = MessageType::AUTONOMOUS_REQUEST;
  msg.header.sequence = seq++;
  msg.header.payload_length = sizeof(Request) - sizeof(MsgHeader);
  msg.header.device_type = 0xCE;
  msg.header.device_id[0] = 0x24; //change your team id
  msg.header.device_id[1] = 0x67; //change your team id
  msg.header.device_id[2] = 0x04; //change your team id

  msg.mission_id = id;
  msg.request = req;

  msg.response = 0x00;
  sprintf(msg.description,"ichthus"); //change your team name
  msg.temporary;
  msg.end_point = 0x0D0A;

  int packet = send(clnt_sock, (char*)&msg, sizeof(msg), 0); 
  if(packet <= 0)
  {
    std::cout << "ERROR[SendRequest] : fail send" << std::endl;
  }
  else
  {
    printf("[SendRequest] id : %d, req : %d", id, req);
    if (req == 0x01)
    {
      printf("(REQ_SELECT_MISSION)\n");
    }
    else if (req == 0x02)
    {
      printf("(REQ_START_POSITION)\n");
    }
    else if (req == 0x03)
    {
      printf("(REQ_END_POSITION)\n");
    }
  }
}

}