/*  ===========================================================================
    Copyright 2022. The ICHTHUS Project. All Rights Reserved.
    Sumin In (ism0705@naver.com),
    Youngjun Han (young@ssu.ac.kr)
    Vision System Laboratory, Soongsil University.
    added by ICHTHUS, Sumin In on 20221026
    [Licensed under the MIT License]  
    ===========================================================================*/

#include "ichthus_v2x/node.hpp"


namespace ichthus_v2x
{

bool done = false;

IchthusV2X::IchthusV2X() : Node("ichthus_v2x") 
{
  DEBUG = this->declare_parameter("debug", true);
  DISPLAY =  this->declare_parameter("display", true);
  IP = this->declare_parameter("ip", "192.168.10.10");
  std::cout << "IP : " << IP << std::endl;

  ConnectSocket();

  auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
  // v2x_pub_ = this->create_publisher<kiapi_msgs::msg::V2xinfo>("v2x_info", qos_profile);
  v2x_pub_ = this->create_publisher<autoware_auto_perception_msgs::msg::TrafficSignalArray>(
            "/external/traffic_light_recognition/traffic_signals", qos_profile);
  loc_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>("/fix", 
            rclcpp::SensorDataQoS().keep_last(64),
            std::bind(&IchthusV2X::GnssCallback, this, std::placeholders::_1));
  vel_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("/v2x", qos_profile,
            std::bind(&IchthusV2X::VelocityCallback, this, std::placeholders::_1));

  if (DEBUG == false)
  {
    gnss_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/gnss_pose", 
                    rclcpp::SensorDataQoS().keep_last(64),
                    std::bind(&IchthusV2X::GnssPoseCallback, this, std::placeholders::_1));
  }
  else
  {
    std::cout << "## DEBUG MODE ##" << std::endl;
    ori_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("/imu/data", qos_profile, /* DEBUG */
                std::bind(&IchthusV2X::ImuCallback, this, std::placeholders::_1));
    pvd_pub_ = this->create_publisher<kiapi_msgs::msg::Pvdinfo>("pvd_info", qos_profile); /* DEBUG */
  }

  if (DISPLAY == true)
  {
    std::cout << "## DISPLAY MODE ##" << std::endl;
  }
}
IchthusV2X::~IchthusV2X()
{
}

void IchthusV2X::GnssCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  Vli_.latitude = msg->latitude*10000000;
  Vli_.longitude = msg->longitude*10000000;
  Vli_.elevation = msg->altitude*10;
}

void IchthusV2X::VelocityCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  Vli_.speed = msg->data[0]*50;
  if (msg->data[1] == 0)
  {
    Vli_.transmisson = 1;
  }
  else if (msg->data[1] == 5)
  {
    Vli_.transmisson = 2;
  }
  else if (msg->data[1] == 6)
  {
    Vli_.transmisson = 0;
  }
  else if (msg->data[1] == 7)
  {
    Vli_.transmisson = 3;
  }
}

void IchthusV2X::GnssPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  double siny_cosp = 2.0 * (msg->pose.orientation.w * msg->pose.orientation.z + msg->pose.orientation.x * msg->pose.orientation.y);
  double cosy_cosp = 1.0 - 2.0 * (msg->pose.orientation.y * msg->pose.orientation.y + msg->pose.orientation.z * msg->pose.orientation.z);
  Vli_.heading = static_cast<double>(static_cast<int>((-(std::atan2(siny_cosp, cosy_cosp) * (180.0 / M_PI)) + 450.0)) % 360)*80;
}

void IchthusV2X::ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  double siny_cosp = 2 * (msg->orientation.w * msg->orientation.z + msg->orientation.x * msg->orientation.y);
  double cosy_cosp = 1 - 2 * (msg->orientation.y * msg->orientation.y + msg->orientation.z * msg->orientation.z);
  Vli_.heading = std::atan2(siny_cosp, cosy_cosp);
}


void IchthusV2X::ConnectSocket()
{
  sockFd = -1;
  std::string obu_ip = IP;
  uint32_t obu_port = PORT;

  struct sockaddr_in obu_addr;
  memset(&obu_addr, 0, sizeof(obu_addr));
  obu_addr.sin_family = AF_INET;
  obu_addr.sin_port = htons(obu_port);

  /* Convert String IP Address */
  if (inet_pton(AF_INET, obu_ip.c_str(), &obu_addr.sin_addr) <= 0)
  {
    std::cout << "ERROR : Error, inet_pton" << std::endl;
    exit(1);
  }

  /* Create Client Socket FD */
  if ((sockFd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0)
  {
    std::cout << "ERROR : connect failed, retry" << std::endl;
    exit(1);
  }

  /* Connect Client, Server TCP */
  int connected = -1;
  if ((connected = connect(sockFd, (const sockaddr *)&obu_addr, sizeof obu_addr))<0)
  {
    std::cout << "ERROR : step sock connect error : " << connected << std::endl;
    exit(1);
  }

  txPvd = GetClockTime(); 

  std::cout << "DEBUG : OBU TCP [" << obu_ip << ":" << obu_port << "] Connected" << std::endl;

  /* Thread */
  th1 = ReceiveSpatThread();
  th2 = SendPvdThread();
}

unsigned long long IchthusV2X::GetClockTime()
{  
  struct timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);  
  uint64_t clock = ts.tv_sec * 1000 + (ts.tv_nsec / 1000000); 
  return clock;
}

thread IchthusV2X::ReceiveSpatThread()
{
  return thread([this]{ReceiveSpat();});
}
thread IchthusV2X::SendPvdThread()
{
  return thread([this]{SendPvd();});
}

void IchthusV2X::ReceiveSpat()
{
  int rxSize = -1;
  while ((rxSize = recv(sockFd, rxBuffer, 2048, MSG_NOSIGNAL)) > 0)
  {
    if(done) 
    {
      close(sockFd);
      exit(-1);
    }
    // std::cout << "Rx " << rxSize << " bytes" << std::endl; /* DEBUG */

    string msgs;
    msgs.append((char *)rxBuffer, sizeof(rxBuffer));

    if (msgs.size() < sizeof(struct CestObuUperPacketHeader))
    { 
      std::cout <<  "require more bytes, current bytes = " << msgs.size() << std::endl;
      break;
    }
      
    std::string payload;
    struct CestObuUperPacketHeader *header = (struct CestObuUperPacketHeader *)&msgs[0]; 
    payload.append(msgs, sizeof(struct CestObuUperPacketHeader), header->payloadLen); 
    // std::cout << "size : " << header->payloadLen << std::endl; /* DEBUG */

    if(header->messageType == 0x3411)
    {
      struct TxWaveUperResultPayload *txpayload = (struct TxWaveUperResultPayload*)&payload[0];
      printf("RX - \"TX_WAVE_UPER_ACK\" [%d/%d/%d]\n", txpayload->txWaveUperSeq, txpayload->resultCode, txpayload->size);
      uperSize = 0;
    }
    else
    {
      printf("RX - \"RX_WAVE_UPER\" [%d]\n", header->payloadLen);
    }

    MessageFrame_t* msgFrame = nullptr;
  
    auto res = uper_decode(0,&asn_DEF_MessageFrame,(void**)&msgFrame, payload.c_str(), header->payloadLen, 0,0);
    switch (res.code)
    {
      case asn_dec_rval_code_e::RC_OK:
          std::cout << "\n[RC_OK]" << std::endl;
          switch (msgFrame->messageId)
          {
            case DSRC_ID_SPAT:
              std::cout << ">> Parse J2735 : SPAT <<" << std::endl;
              if (DISPLAY == true)
              {
                PrintSpat((SPAT_t*)&msgFrame->value.choice.SPAT);
              } 
              PublishSpat((SPAT_t*)&msgFrame->value.choice.SPAT);
              break;
            case DSRC_ID_MAP:
              std::cout << ">> Parse J2735 : MAP <<" << std::endl;
            default:
                break;
          }
          //asn_fprint(stderr, rxUperBuffer&asn_DEF_MessageFrame, msgFrame); /* DEBUG */
          break;
      case asn_dec_rval_code_e::RC_WMORE:
          std::cout << "\n[RC_WMORE]" << std::endl;
          break;
      case asn_dec_rval_code_e::RC_FAIL:
          std::cout << "\n[RC_FAIL]" << std::endl;
          break;
      default:
          break;
    }      
    msgs.erase(0, sizeof(struct CestObuUperPacketHeader) + payload.size());
    usleep(1000);
  }
  close(sockFd);
}

void IchthusV2X::PublishSpat(SPAT_t *spat)
{
  autoware_auto_perception_msgs::msg::TrafficSignalArray v2x_msg;

  rclcpp::Time current_time = this->now();
  v2x_msg.header.stamp = current_time;
  v2x_msg.header.frame_id = ' ';

  std::cout << "\ninit_gen_SPaT" << std::endl;
  for (int i=0; i<spat->intersections.list.count; i++)
  {
    struct IntersectionState *inter_p = spat->intersections.list.array[i];
    for (int j=0 ; j<inter_p->states.list.count; ++j)
    {
      autoware_auto_perception_msgs::msg::TrafficSignal signal;
      struct MovementState *move_p = inter_p->states.list.array[j];
      if (inter_p->id.id == 1)
      {
        switch (move_p->signalGroup)
        {
          case 13:
          case 14:
            signal.map_primitive_id = 1753;
            break;
          case 16:
          case 17:
            signal.map_primitive_id = 1648;
            break;
          case 19:
          case 20:
            signal.map_primitive_id = 1742;
            break;
          case 22:
          case 23:
            signal.map_primitive_id = 1718;
            break;
        }
      }
      else if (inter_p->id.id == 2)
      {
        switch (move_p->signalGroup)
        {
          case 13:
          case 14:
            signal.map_primitive_id = 1707;
            break;
          case 16:
          case 17:
            signal.map_primitive_id = 1660;
            break;
          case 19:
          case 20:
            signal.map_primitive_id = 1696;
            break;
          case 22:
          case 23:
            signal.map_primitive_id = 1684;
            break;
        }
      }
      else if (inter_p->id.id == 3)
      {
        switch (move_p->signalGroup)
        {
          // case 17:
          //   signal.map_primitive_id = 1764;
          //   break;
          case 19:
            signal.map_primitive_id = 1878;
        }
      }

      for(int k=0; k<move_p->state_time_speed.list.count; ++k)
      {
        autoware_auto_perception_msgs::msg::TrafficLight light;
        struct MovementEvent *mvevent_p = move_p->state_time_speed.list.array[k];
        /* RED & YELLOW */
        if (mvevent_p->eventState == 3 || mvevent_p->eventState == 7 || mvevent_p->eventState == 8)
        {
          light.color = 1;
        }
        /* GREEN */
        else if (mvevent_p->eventState == 5 || mvevent_p->eventState == 6)
        {
          light.color = 3;
        }
        light.shape = 5;
        light.status = 14;

        signal.lights.emplace_back(light);
      }
      v2x_msg.signals.emplace_back(signal);
    }
  }
  v2x_pub_->publish(v2x_msg);
}

void IchthusV2X::PrintSpat(SPAT_t *spat)
{ 
  for (int i = 0; i < spat->intersections.list.count; i++)
  {
    struct IntersectionState *ptr = spat->intersections.list.array[i];

    for (int i = 0; i < static_cast<int>(ptr->name->size); ++i)
    {
      printf("%c", ptr->name->buf[i]);
    }

    printf("\nid\n");
    printf("  region : %ld\n", *ptr->id.region);
    printf("  id     : %ld\n", ptr->id.id);
    printf("revision     : %ld\n", ptr->revision);

    printf("status       : 0x");
    for (int i = 0; i < static_cast<int>(ptr->status.size); ++i)
    {
      printf("%02X", ptr->status.buf[i]);
    }
    printf("\n");

    // printf("      revision     : %ld\n", ptr->revision);
    printf("moy          : %ld\n", *ptr->moy);
    printf("timeStamp    : %ld\n", *ptr->timeStamp);
      
    // if (ptr->enabledLanes) 
    // {
    //   printf("      enabledLanes : %ld\n", ptr->enabledLanes);
    // }
      
    if (ptr->states.list.count) 
    {
      printf("states       : %d items\n", ptr->states.list.count);
        
      for (int j = 0; j < ptr->states.list.count; ++j) 
      {
        printf("  states[%d]\n", j);
        printf("    movementName       : ");
          
        for (int l = 0; l < static_cast<int>(ptr->states.list.array[j]->movementName->size); ++l) 
        {
          printf("%c", ptr->states.list.array[j]->movementName->buf[l]);
        }
          
        printf("\n");
        printf("    signalGroup        : %ld\n", ptr->states.list.array[j]->signalGroup);
          
        if (ptr->states.list.array[j]->state_time_speed.list.count) 
        {
          printf("    state-time-speed   : %d items\n", ptr->states.list.array[j]->state_time_speed.list.count);
            
          for (int k = 0; k < ptr->states.list.array[j]->state_time_speed.list.count; ++k) 
          {
            struct MovementEvent *mvevent_p = ptr->states.list.array[j]->state_time_speed.list.array[k];
            
            printf("      Item %d\n", k);
            printf("        MovementEvent\n");
            printf("          eventState(%ld) : ", mvevent_p->eventState);
            switch (mvevent_p->eventState)
            {
							case e_MovementPhaseState::MovementPhaseState_unavailable:
                printf("unavailable\n");
                break;
							case e_MovementPhaseState::MovementPhaseState_dark:
                printf("dark\n");
                break;
							case e_MovementPhaseState::MovementPhaseState_stop_Then_Proceed:
                printf("stop_Then_Proceeddark\n");
                break;
							case e_MovementPhaseState::MovementPhaseState_stop_And_Remain:
                printf("stop_And_Remain\n");
                break;
							case e_MovementPhaseState::MovementPhaseState_pre_Movement:
                printf("pre_Movement\n");
                break;
							case e_MovementPhaseState::MovementPhaseState_permissive_Movement_Allowed:
								printf("permissive_Movement_Allowed\n");
                break;
							case e_MovementPhaseState::MovementPhaseState_protected_Movement_Allowed:
								printf("protected_Movement_Allowed\n");
                break;
							case e_MovementPhaseState::MovementPhaseState_permissive_clearance:
								printf("permissive_clearance\n");
                break;
							case e_MovementPhaseState::MovementPhaseState_protected_clearance:
								printf("protected_clearance\n");
                break;
							case e_MovementPhaseState::MovementPhaseState_caution_Conflicting_Traffic:
								printf("caution_Conflicting_Traffic\n");
                break;
							default:
								printf("Impossible situation happened. please check this\n");
                exit(-1);
						}
            printf("          timing\n");
            printf("            minEndTime : %ld\n", ptr->states.list.array[j]->state_time_speed.list.array[k]->timing->minEndTime);
          }
        }
        // fprintf(stdout, "          regional           : %ld\n", ptr->states.list.array[j]->regional);
      
        if (ptr->states.list.array[j]->maneuverAssistList->list.count)
        {
          printf("    maneuverAssistList : %d items\n", ptr->states.list.array[j]->maneuverAssistList->list.count);
        
          for (int j = 0; j < ptr->states.list.array[j]->maneuverAssistList->list.count; ++j) 
          {
            printf("      maneuverAssistList[%d]\n", j);
            printf("        connectionID     : %ld\n",  ptr->states.list.array[j]->maneuverAssistList->list.array[j]->connectionID);

            if (ptr->states.list.array[j]->maneuverAssistList->list.array[j]->pedBicycleDetect)
            {
              if (*ptr->states.list.array[j]->maneuverAssistList->list.array[j]->pedBicycleDetect == true)
              {
                printf("        pedBicycleDetect : True\n");
              }
              else
              {
                printf("        pedBicycleDetect : False\n");
              }
            }
          }
        }
      }
    }	
  }
}


/* TX_WAVE_UPER(RESULT) */
void IchthusV2X::SendPvd()
{
  while(1)
  {
    if(done) 
    {
      close(sockFd);
      exit(-1);
    }
    
    if(V2iPvd() < 0)
    {
      std::cout << "ERROR : disconnect TCP, retry" << std::endl;
      close(sockFd);
      exit(1);
    }

    usleep(100000);
  }
}

int IchthusV2X::V2iPvd()
{
  unsigned long long interval = GetClockTime() - txPvd;  // msec;

  if(interval < PVD_INTERVAL)
  {
    return 0;
  }

  txPvd += (interval - interval%PVD_INTERVAL); 

  MessageFrame_t msg;
  char uper[MAX_UPER_SIZE]; 

  FillPvd(&msg);

  std::cout << ">> Parse J2735 : PVD <<" << std::endl;
  if (DISPLAY == false)
  {
    PrintPvd(&msg);
  }

  PublishPvd();               /* DEBUG */

  int encodedBits = EncodeJ2735Uper(uper, &msg);
  // printf("bits:%d\n",encodedBits); /*DEBUG */

  /* Encdoing Fail, unable to send */
  if(encodedBits < 0)
  {
    return 0;
  }
    
  int byteLen = encodedBits / 8 + ((encodedBits % 8)? 1:0);
  // print_hex(uper,byteLen); /* DEBUG */    
      
  return SendRequest(uper,byteLen);  
}

void IchthusV2X::PrintHex(char *data, int len)
{
  std::cout << "HEX[" << len << "] : ";
  for(int i = 0 ; i < len ; i++)
  {
    printf("%02X",(data[i] & 0xFF));
  }
  printf("\n");
}

void IchthusV2X::FillPvd(MessageFrame_t *dst)
{
  /* current day, time */
  time_t timer;
  struct tm *t;
  timer = time(NULL);
  t = localtime(&timer);

  // ASN_STRUCT_RESET(asn_DEF_MessageFrame, dst); /* DEBUG */

  dst->messageId = 26; // Reference J2735.pdf - DE_DSRC_MessageID, probeVehicleData DSRCmsgID ::= 26 -- PVD 
  dst->value.present = MessageFrame__value_PR_ProbeVehicleData; // MessageFrame::value choice (asn1c)

  ProbeVehicleData_t *ptrPvd = &dst->value.choice.ProbeVehicleData;

  ptrPvd->timeStamp = NULL; // OPTIONAL, not to use
  ptrPvd->segNum = NULL;    // OPTIONAL, not to use
  ptrPvd->regional = NULL;  // OPTIONAL, not to use

  ptrPvd->probeID = new(struct VehicleIdent);
  ptrPvd->probeID->name = NULL;         // OPTIONAL, not to use
  ptrPvd->probeID->ownerCode = NULL;    // OPTIONAL, not to use
  ptrPvd->probeID->vehicleClass = NULL; // OPTIONAL, not to use
  ptrPvd->probeID->vin = NULL;          // OPTIONAL, not to use
  ptrPvd->probeID->vehicleType = NULL;  // OPTIONAL, not to use

  ptrPvd->probeID->id = new(struct VehicleID);
  ptrPvd->probeID->id->present = VehicleID_PR_entityID;   
  ptrPvd->probeID->id->present = VehicleID_PR_entityID;
  ptrPvd->probeID->id->choice.entityID.buf = (unsigned char *)malloc(4);
  ptrPvd->probeID->id->choice.entityID.size = 4; 
  ptrPvd->probeID->id->choice.entityID.buf[0] = 0xCE; // (INPUT) <---- 할당된 대학별 ID 입력
  ptrPvd->probeID->id->choice.entityID.buf[1] = 0x24; // (INPUT) <---- 할당된 대학별 ID 입력
  ptrPvd->probeID->id->choice.entityID.buf[2] = 0x67; // (INPUT) <---- 할당된 대학별 ID 입력
  ptrPvd->probeID->id->choice.entityID.buf[3] = 0x04; // (INPUT) <---- 할당된 대학별 ID 입력

  ptrPvd->startVector.utcTime = new(struct DDateTime);  
  ptrPvd->startVector.utcTime->year = new DYear_t;
  ptrPvd->startVector.utcTime->month = new DMonth_t; 
  ptrPvd->startVector.utcTime->day = new DDay_t; 
  ptrPvd->startVector.utcTime->hour = new DHour_t; 
  ptrPvd->startVector.utcTime->minute = new DMinute_t; 
  ptrPvd->startVector.utcTime->second = new DSecond_t;  
  ptrPvd->startVector.utcTime->offset = NULL; // OPTIONAL, not to use

  *ptrPvd->startVector.utcTime->year = t->tm_year+1900; // (INPUT) <--------------- 현재 UTC 시간 입력 (년도)
  *ptrPvd->startVector.utcTime->month = t->tm_mon+1;    // (INPUT) <--------------- 현재 UTC 시간 입력 (년도)
  *ptrPvd->startVector.utcTime->day = t->tm_mday;       // (INPUT) <--------------- 현재 UTC 시간 입력 (년도)
  *ptrPvd->startVector.utcTime->hour = t->tm_hour;      // (INPUT) <--------------- 현재 UTC 시간 입력 (년도)
  *ptrPvd->startVector.utcTime->minute = t->tm_min;     // (INPUT) <--------------- 현재 UTC 시간 입력 (년도)
  *ptrPvd->startVector.utcTime->second = t->tm_sec;     // (INPUT) <--------------- 현재 UTC 시간 입력 (년도)

  ptrPvd->startVector.elevation = new DSRC_Elevation_t;
  ptrPvd->startVector.heading = new Heading_t;
  ptrPvd->startVector.speed = new (struct TransmissionAndSpeed);
  ptrPvd->startVector.posAccuracy = NULL;     // OPTIONAL, not to use
  ptrPvd->startVector.posConfidence = NULL;   // OPTIONAL, not to use
  ptrPvd->startVector.timeConfidence = NULL;  // OPTIONAL, not to use
  ptrPvd->startVector.speedConfidence = NULL; // OPTIONAL, not to use

  ptrPvd->startVector.Long = Vli_.longitude;                // (INPUT) <--------------- 현재 차량의 위치 (위도) (Longitude, DD 좌표계)
  ptrPvd->startVector.lat = Vli_.latitude;                  // (INPUT) <--------------- 현재 차량의 위치 (경도) (Latitude,  DD 좌표계)
  *ptrPvd->startVector.elevation = Vli_.elevation;          // (INPUT) <--------------- 현재 차량의 위치 (고도) (Elevation)   
  *ptrPvd->startVector.heading = Vli_.heading;              // (INPUT) <--------------- 현재 차량의 주행 방향 (북쪽 0도)           
  ptrPvd->startVector.speed->speed = Vli_.speed;            // (INPUT) <--------------- 현재 차량의 속도        
  ptrPvd->startVector.speed->transmisson = Vli_.transmisson;// (INPUT) <--------------- 현재 차량의 변속기 상태          

  ptrPvd->vehicleType.hpmsType = new VehicleType_t;
  ptrPvd->vehicleType.keyType = NULL;       // OPTIONAL, not to use
  ptrPvd->vehicleType.fuelType = NULL;      // OPTIONAL, not to use
  ptrPvd->vehicleType.iso3883 = NULL;       // OPTIONAL, not to use
  ptrPvd->vehicleType.regional = NULL;      // OPTIONAL, not to use
  ptrPvd->vehicleType.responderType = NULL; // OPTIONAL, not to use
  ptrPvd->vehicleType.responseEquip = NULL; // OPTIONAL, not to use
  ptrPvd->vehicleType.role = NULL;          // OPTIONAL, not to use
  ptrPvd->vehicleType.vehicleType = NULL;   // OPTIONAL, not to use
  *ptrPvd->vehicleType.hpmsType = VehicleType_car; 

  ptrPvd->snapshots.list.count = 1; 
  ptrPvd->snapshots.list.array = new (struct Snapshot *);
  ptrPvd->snapshots.list.array[0] = new (struct Snapshot);
  struct Snapshot *ptrSnapshot = ptrPvd->snapshots.list.array[0]; 

  ptrSnapshot->thePosition.utcTime = new (struct DDateTime);
  ptrSnapshot->thePosition.utcTime->year = new DYear_t;
  ptrSnapshot->thePosition.utcTime->month = new DMonth_t;
  ptrSnapshot->thePosition.utcTime->day = new DDay_t;
  ptrSnapshot->thePosition.utcTime->hour = new DHour_t;
  ptrSnapshot->thePosition.utcTime->minute = new DMinute_t;
  ptrSnapshot->thePosition.utcTime->second = new DSecond_t;
  ptrSnapshot->thePosition.utcTime->offset = NULL; // OPTIONAL, not to use

  ptrSnapshot->thePosition.elevation = new DSRC_Elevation_t;
  ptrSnapshot->thePosition.speed = new (struct TransmissionAndSpeed);
  ptrSnapshot->thePosition.heading = new Heading_t;
  ptrSnapshot->thePosition.posAccuracy = NULL;     // OPTIONAL, not to use
  ptrSnapshot->thePosition.posConfidence = NULL;   // OPTIONAL, not to use
  ptrSnapshot->thePosition.timeConfidence = NULL;  // OPTIONAL, not to use
  ptrSnapshot->thePosition.speedConfidence = NULL; // OPTIONAL, not to use

  *ptrSnapshot->thePosition.utcTime->year = t->tm_year+1900; // (INPUT) <--------------- 직전 전송한 PVD의 UTC 시간 입력 (년도)
  *ptrSnapshot->thePosition.utcTime->month = t->tm_mon+1;    // (INPUT) <--------------- 직전 전송한 PVD의 UTC 시간 입력 (월)
  *ptrSnapshot->thePosition.utcTime->day = t->tm_mday;       // (INPUT) <--------------- 직전 전송한 PVD의 UTC 시간 입력 (일)
  *ptrSnapshot->thePosition.utcTime->hour = t->tm_hour;      // (INPUT) <--------------- 직전 전송한 PVD의 UTC 시간 입력 (시)
  *ptrSnapshot->thePosition.utcTime->minute = t->tm_min;     // (INPUT) <--------------- 직전 전송한 PVD의 UTC 시간 입력 (분)
  *ptrSnapshot->thePosition.utcTime->second = t->tm_sec;     // (INPUT) <--------------- 직전 전송한 PVD의 UTC 시간 입력 (초)
  
  ptrSnapshot->thePosition.lat = Vli_.latitude;                   // (INPUT) <--------------- 현재 차량의 위치 (위도) (Longitude, DD 좌표계)
  ptrSnapshot->thePosition.Long = Vli_.longitude;                 // (INPUT) <--------------- 현재 차량의 위치 (경도) (Latitude,  DD 좌표계) 
  *ptrSnapshot->thePosition.elevation = Vli_.elevation;           // (INPUT) <--------------- 현재 차량의 위치 (고도) (Elevation)   
  *ptrSnapshot->thePosition.heading = Vli_.heading;               // (INPUT) <--------------- 현재 차량의 주행 방향 (북쪽 0도)               
  ptrSnapshot->thePosition.speed->speed = Vli_.speed;             // (INPUT) <-------------- -현재 차량의 속도                  
  ptrSnapshot->thePosition.speed->transmisson = Vli_.transmisson; // (INPUT) <--------------- 현재 차량의 변속기 상태          
}

/* DEBUG */
void IchthusV2X::PublishPvd()
{
  kiapi_msgs::msg::Pvdinfo pvd_msg;
  pvd_msg.latitude = Vli_.latitude;
  pvd_msg.longitude = Vli_.longitude;
  pvd_pub_->publish(pvd_msg);
}

void IchthusV2X::PrintPvd(MessageFrame_t *pvd)
{
  ProbeVehicleData_t *ptrPvd = &pvd->value.choice.ProbeVehicleData;

  printf("ProbeID\n");
  printf("  id\n");
  printf("    entityID    : %02X:%02X:%02X:%02X\n", ptrPvd->probeID->id->choice.entityID.buf[0],
                                                    ptrPvd->probeID->id->choice.entityID.buf[1],
                                                    ptrPvd->probeID->id->choice.entityID.buf[2],
                                                    ptrPvd->probeID->id->choice.entityID.buf[3]);
  printf("startVector\n");
  printf("  utcTime\n");
  printf("    year        : %ld\n", *ptrPvd->startVector.utcTime->year);
  printf("    month       : %ld\n", *ptrPvd->startVector.utcTime->month);
  printf("    day         : %ld\n", *ptrPvd->startVector.utcTime->day);
  printf("    hour        : %ld\n", *ptrPvd->startVector.utcTime->hour);
  printf("    minute      : %ld\n", *ptrPvd->startVector.utcTime->minute);
  printf("    second      : %ld\n", *ptrPvd->startVector.utcTime->second);
  printf("  Long      : %ld\n", ptrPvd->startVector.Long);
  printf("  lat       : %ld\n", ptrPvd->startVector.lat);
  printf("  elevation : %ld\n", *ptrPvd->startVector.elevation);
  printf("  heading   : %ld\n", *ptrPvd->startVector.heading);
  printf("  speed\n");
  printf("    speed       : %ld\n", ptrPvd->startVector.speed->speed);
  
  printf("    transmisson : ");
  if (ptrPvd->startVector.speed->transmisson == 0)
  {
    printf("Neutral");
  }
  else if (ptrPvd->startVector.speed->transmisson == 1)
  {
    printf("Park");
  }
  else if (ptrPvd->startVector.speed->transmisson == 2)
  {
    printf("Forward gears");
  }
  else if (ptrPvd->startVector.speed->transmisson == 3)
  {
    printf("Reverse gears");
  }
  printf("(%ld)\n", ptrPvd->startVector.speed->transmisson);

  printf("vehicleTiype\n");
  switch(*ptrPvd->vehicleType.hpmsType)
  {
    case e_VehicleType::VehicleType_car:
      printf("  hpmsType  : VehicleType_car(4)\n");
  }
  
  for (int i = 0; i < ptrPvd->snapshots.list.count; i++)
  {
    struct Snapshot *ptrSnapshot = ptrPvd->snapshots.list.array[i];

    printf("snapshots[%d]\n", i);
    printf("  thePosition\n");
    printf("    utcTime\n");
    printf("      year   : %ld\n", *ptrSnapshot->thePosition.utcTime->year);
    printf("      month  : %ld\n", *ptrSnapshot->thePosition.utcTime->month);
    printf("      day    : %ld\n", *ptrSnapshot->thePosition.utcTime->day);
    printf("      hour   : %ld\n", *ptrSnapshot->thePosition.utcTime->hour);
    printf("      minute : %ld\n", *ptrSnapshot->thePosition.utcTime->minute);
    printf("      second : %ld\n", *ptrSnapshot->thePosition.utcTime->second); 
    printf("  Long      : %ld\n", ptrSnapshot->thePosition.Long);
    printf("  lat       : %ld\n", ptrSnapshot->thePosition.lat);
    printf("  elevation : %ld\n", *ptrSnapshot->thePosition.elevation);
    printf("  heading   : %ld\n", *ptrSnapshot->thePosition.heading);
    printf("  speed\n");
    printf("    speed       : %ld\n", ptrSnapshot->thePosition.speed->speed);
    printf("    transmisson : ");
    if (ptrSnapshot->thePosition.speed->transmisson == 0)
    {
      printf("Neutral");
    }
    else if (ptrSnapshot->thePosition.speed->transmisson == 1)
    {
      printf("Park");
    }
    else if (ptrSnapshot->thePosition.speed->transmisson == 2)
    {
      printf("Forward gears");
    }
    else if (ptrSnapshot->thePosition.speed->transmisson == 3)
    {
      printf("Reverse gears");
    }
    printf("(%ld)\n", ptrSnapshot->thePosition.speed->transmisson);  
  }
}

int IchthusV2X::EncodeJ2735Uper(char *dst, MessageFrame_t *src)
{
  asn_enc_rval_t ret = uper_encode_to_buffer(&asn_DEF_MessageFrame,
                                              NULL,
                                              src,
                                              dst, MAX_UPER_SIZE);

  /* UPER Encoding Success */ 
  if (ret.encoded > 0)
  {
    return ret.encoded;
  }
  /* UPER Encoding Failed */
  else
  { 
    if (ret.failed_type != NULL)
    {
      std::cout << "encoded error value name = " << ret.failed_type->name << std::endl;
    }
    return -1;
  }
}

int IchthusV2X::SendRequest(char *uper, unsigned short uperLength)
{
  if(sockFd < 0)
  {
    return -1;
  }
  int packetLen = uperLength + sizeof(struct CestObuUperPacketHeader);

  char packet[OBU_RECEIVE_BUFFER_SIZE];  // tcp header size + uper binary size 

  struct CestObuUperPacketHeader *ptrHeader = (struct CestObuUperPacketHeader *)&packet[0];
  ptrHeader->messageType = 0x4311; // TX_WAVE_UPER
  ptrHeader->seq = packetSeq++;
  ptrHeader->payloadLen = uperLength;
  ptrHeader->deviceType = 0xCE;
  memcpy(ptrHeader->deviceId,clientDeviceId,3);
  memcpy(packet + sizeof(struct CestObuUperPacketHeader), uper, uperLength);

  /* PVD Hz DEBUG */
  // time_t timer1;
  // struct tm *t1;
  // timer1 = time(NULL);
  // RCLCPP_INFO(this->get_logger(), "send time : %ld\n",timer1);
  // std::cout<<"send time : "<<rclcpp::Clock().now().nanoseconds()<<std::endl;

  int res = write(sockFd, packet, packetLen);
  if (res > 0)
  {
    printf("TX - \"TX_WAVE_UPER\" SEQ[%d] = ", ptrHeader->seq);
    PrintHex(uper, uperLength);

    if (res != packetLen)
    {            
      std::cout << "DEBUG tcp tx purge" << std::endl;
      return -1;
    }
    else
    {
      return res;
    }
  }

  return 0;
}

}