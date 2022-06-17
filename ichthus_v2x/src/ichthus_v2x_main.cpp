#include "ichthus_v2x/ichthus_v2x.hpp"


namespace ichthus_v2x
{

bool done = false;
bool is_first_topic = true;

IchthusV2X::IchthusV2X() : Node("ichthus_v2x") 
{
  auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
  v2x_pub_ = this->create_publisher<kiapi_msgs::msg::V2xinfo>("v2x_info", qos_profile);
  // pvd_pub_ = this->create_publisher<kiapi_msgs::msg::Pvdinfo>("pvd_info", qos_profile); /* for test */
  // loc_sub_ = this->create_subscription<kiapi_msgs::msg::Mylocation>("gnss_info", qos_profile,
              // std::bind(&IchthusV2X::LocationCallback, this, std::placeholders::_1));   /* for test */
  loc_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>("/fix", qos_profile,
              std::bind(&IchthusV2X::GnssCallback, this, std::placeholders::_1));
  ori_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("/imu/data", qos_profile,
              std::bind(&IchthusV2X::ImuCallback, this, std::placeholders::_1));
  vel_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("/v2x", qos_profile,
              std::bind(&IchthusV2X::VelocityCallback, this, std::placeholders::_1));
}
IchthusV2X::~IchthusV2X()
{
}
/*
void IchthusV2X::LocationCallback(const kiapi_msgs::msg::Mylocation::SharedPtr msg)
{
  std::cout << "callback" << std::endl;
  Vli_.latitude = msg->latitude;
  Vli_.longitude = msg->longitude;
  Vli_.elevation = msg->elevation;
  Vli_.heading = msg->heading;
  Vli_.speed = msg->speed;
  Vli_.transmisson = msg->transmisson;

  if(is_first_topic)
  {
    is_first_topic = false;
    connect_obu_uper_tcp();
  }
}
*/
void IchthusV2X::GnssCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  // std::cout << "callback" << std::endl;
  Vli_.latitude = msg->latitude;
  Vli_.longitude = msg->longitude;
  Vli_.elevation = msg->altitude;

  if(is_first_topic)
  {
    is_first_topic = false;
    connect_obu_uper_tcp();
  }
}

void IchthusV2X::ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  // std::cout << "callback" << std::endl;
  double siny_cosp = 2 * (msg->orientation.w * msg->orientation.z + msg->orientation.x * msg->orientation.y);
  double cosy_cosp = 1 - 2 * (msg->orientation.y * msg->orientation.y + msg->orientation.z * msg->orientation.z);
  Vli_.heading = std::atan2(siny_cosp, cosy_cosp);
}

void IchthusV2X::VelocityCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  // std::cout << "velcallback1 : " << msg->data[0] << std::endl;
  std::cout << "velcallback2 : " << msg->data[1] << std::endl;
  Vli_.speed = msg->data[0];
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

void IchthusV2X::connect_obu_uper_tcp()
{
  sockFd = -1;
  std::string obu_ip = IP;
  uint32_t obu_port = PORT;

  struct sockaddr_in obu_addr;
  memset(&obu_addr, 0, sizeof(obu_addr));
  obu_addr.sin_family = AF_INET;
  obu_addr.sin_port = htons(obu_port);

  // String IP 주소 변환 
  if (inet_pton(AF_INET, obu_ip.c_str(), &obu_addr.sin_addr) <= 0)
  {
    // fprintf(stderr, "DEBUG : Error, inet_pton\n");
    std::cout << "DEBUG : Error, inet_pton" << std::endl;
    exit(1);
  }

  // 클라이언트 Socket FD 생성
  if ((sockFd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0)
  {
    // fprintf(stderr, "DEBUG : connect failed, retry\n");
    std::cout << "DEBUG : connect failed, retry" << std::endl;
    exit(1);
  }

  // 클라이언트, 서버 TCP 연결
  int connected = -1;
  if ((connected = connect(sockFd, (const sockaddr *)&obu_addr, sizeof obu_addr))<0)
  {
    // fprintf(stderr, "DEBUG : step sock connect error : %d\n",connected);
    std::cout << "DEBUG : step sock connect error : " << connected << std::endl;
    exit(1);
  }

  txPvd = get_clock_time(); 

  std::cout << "DEBUG : OBU TCP [" << obu_ip << ":" << obu_port << "] Connected" << std::endl;

  th1 = receiveSpatThread();
  th2 = sendPvdThread();
}

unsigned long long IchthusV2X::get_clock_time()
{  
  struct timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);  
  uint64_t clock = ts.tv_sec * 1000 + (ts.tv_nsec / 1000000); 
  return clock;
}

thread IchthusV2X::receiveSpatThread()
{
  return thread([this]{receiveSpat();});
}
thread IchthusV2X::sendPvdThread()
{
  return thread([this]{sendPvd();});
}

void IchthusV2X::receiveSpat()
{
  int rxSize = -1;
  while ((rxSize = recv(sockFd, rxBuffer, 2048, MSG_NOSIGNAL)) > 0)
  {
    if(done) 
    {
      close(sockFd);
      exit(-1);
    }
    // fprintf(stdout, "Rx %d bytes\n", rxSize);

    string msgs;
    msgs.append((char *)rxBuffer, sizeof(rxBuffer));
    // std::cout << "msgs : " << msgs << std:: endl;

    if (msgs.size() < sizeof(struct CestObuUperPacketHeader))
    { 
        fprintf(stdout, "require more bytes, current bytes = %d \n",msgs.size());
        break;
    }
      
    std::string payload;
    struct CestObuUperPacketHeader *header = (struct CestObuUperPacketHeader *)&msgs[0]; 
    payload.append(msgs, sizeof(struct CestObuUperPacketHeader), header->payloadLen); 
    // std::cout<<"size : "<<header->payloadLen<<endl;

    if(header->messageType == 0x3411)
    {
      struct TxWaveUperResultPayload *txpayload = (struct TxWaveUperResultPayload*)&payload[0];
      // std::cout << "RX - \"TX_WAVE_UPER_ACK\" [" << txpayload->txWaveUperSeq << "/" << txpayload->resultCode << "/" << txpayload->size << "]" << std::endl;
      printf("RX - \"TX_WAVE_UPER_ACK\" [%d/%d/%d]\n", txpayload->txWaveUperSeq, txpayload->resultCode, txpayload->size);
      uperSize = 0;
    }
    else
    {
      std::cout << "RX - \"RX_WAVE_UPER\" [" << header->payloadLen << "]" << std::endl;
    }

    MessageFrame_t* msgFrame = nullptr;
  
    auto res = uper_decode(0,&asn_DEF_MessageFrame,(void**)&msgFrame, payload.c_str(), header->payloadLen, 0,0);
    switch (res.code)
    {
      case asn_dec_rval_code_e::RC_OK:
          fprintf(stderr,"\n[RC_OK]\n");
          switch (msgFrame->messageId)
          {
            case DSRC_ID_SPAT:
              std::cout << ">> Parse J2735 : SPAT <<" << endl;
              parse_spat((SPAT_t*)&msgFrame->value.choice.SPAT); 
              gen_SPaTMsg((SPAT_t*)&msgFrame->value.choice.SPAT);
              break;

            case DSRC_ID_MAP:
              std::cout << ">> Parse J2735 : MAP <<" << endl;

            default:
                break;
          }

          v2x_pub_->publish(v2x_msg);
          //asn_fprint(stderr, rxUperBuffer&asn_DEF_MessageFrame, msgFrame);

          break;
      case asn_dec_rval_code_e::RC_WMORE:
          fprintf(stderr,"\n[RC_WMORE]\n");
          break;
  
      case asn_dec_rval_code_e::RC_FAIL:
          fprintf(stderr,"\n[RC_FAIL]\n");
          break;

      default:
          break;
    }
      
    msgs.erase(0, sizeof(struct CestObuUperPacketHeader) + payload.size());
    // usleep(1000);
  }
  close(sockFd);
}

void IchthusV2X::gen_SPaTMsg(SPAT_t *spat)
{
  v2x_msg.msg_type = kiapi_msgs::msg::V2xinfo::SPAT_MSG_TYPE;

  v2x_msg.spat_id_region = 0;            // 교차로 식별 고유 ID
  v2x_msg.spat_movement_cnt = 0;
  v2x_msg.spat_movement_name.resize(0);  // 교차로 명칭
  v2x_msg.spat_signal_group.resize(0);   // 메시지에 해당되는 차선목록
  v2x_msg.spat_eventstate.resize(0);     // 빨,초,노 event값(신호상태)
  v2x_msg.spat_minendtime.resize(0);     // 만약 minEndTime 존재한다면 넣기.(예상 최소 종료 시각)

  // ROS_INFO("init_gen_SPaT");
  // RCLCPP_INFO("init_gen_SPaT");
  std::cout << "init_gen_SPaT" << endl;
  for(int i = 0; i < spat->intersections.list.count; i++)
  {
    struct IntersectionState *inter_p = spat->intersections.list.array[i];
    v2x_msg.spat_id_region = inter_p->id.id;
    v2x_msg.spat_movement_cnt = inter_p->states.list.count;

    for (int j = 0 ; j < v2x_msg.spat_movement_cnt; ++j)
    {
      struct MovementState *move_p = inter_p->states.list.array[j];
      v2x_msg.spat_signal_group.push_back(move_p->signalGroup); 
      v2x_msg.spat_movement_name.push_back((char*)move_p->movementName->buf);
    
      for(int k = 0; k < move_p->state_time_speed.list.count; ++k)
      {
        struct MovementEvent *mvevent_p = move_p->state_time_speed.list.array[k];
        v2x_msg.spat_eventstate.push_back(mvevent_p->eventState);
          
        if (mvevent_p->timing)
        {
          v2x_msg.spat_minendtime.push_back(mvevent_p->timing->minEndTime);
        }
      }
    }
  }	
}

int IchthusV2X::parse_spat(SPAT_t *spat)
{ 
  for (int i = 0; i < spat->intersections.list.count; i++)
  {
    struct IntersectionState *ptr = spat->intersections.list.array[i];

    // fprintf(stdout, "   Item %ld\n", i);
    for (int i = 0; i < ptr->name->size; ++i)
    {
      printf("%c", ptr->name->buf[i]);
    }

    printf("\n");

    fprintf(stdout, "id\n");
    fprintf(stdout, "  region : %ld\n", ptr->id.region);
    fprintf(stdout, "  id     : %ld\n", ptr->id.id);
    fprintf(stdout, "revision     : %ld\n", ptr->revision);

    fprintf(stdout, "status       : ");
    for (int i = 0; i < ptr->status.size; ++i)
    {
      printf("%02X", ptr->status.buf[i]);
    }
    printf("\n");

    // fprintf(stdout, "      revision     : %ld\n", ptr->revision);
    fprintf(stdout, "moy          : %ld\n", *ptr->moy);
    fprintf(stdout, "timeStamp    : %ld\n", *ptr->timeStamp);
      
    if (ptr->enabledLanes) 
    {
      fprintf(stdout, "      enabledLanes : %ld\n", ptr->enabledLanes);
    }
      
    if (ptr->states.list.count) 
    {
      fprintf(stdout, "states       : %ld items\n", ptr->states.list.count);
        
      for (int j = 0; j < ptr->states.list.count; ++j) 
      {
        fprintf(stdout, "  states[%ld]\n", j);
        fprintf(stdout, "    movementName       : ");
          
        for (int l = 0; l < ptr->states.list.array[j]->movementName->size; ++l) 
        {
          fprintf(stdout, "%c", ptr->states.list.array[j]->movementName->buf[l]);
        }
          
        fprintf(stdout, "\n");
        fprintf(stdout, "    signalGroup        : %ld\n", ptr->states.list.array[j]->signalGroup);
          
        if (ptr->states.list.array[j]->state_time_speed.list.count) 
        {
          fprintf(stdout, "    state-time-speed   : %ld items\n", ptr->states.list.array[j]->state_time_speed.list.count);
            
          for (int k = 0; k < ptr->states.list.array[j]->state_time_speed.list.count; ++k) 
          {
            struct MovementEvent *mvevent_p = ptr->states.list.array[j]->state_time_speed.list.array[k];
            
            fprintf(stdout, "      Item %ld\n", k);
            fprintf(stdout, "        MovementEvent\n");
            fprintf(stdout, "          eventState(%ld) : ", mvevent_p->eventState);
            switch (mvevent_p->eventState)
            {
							case e_MovementPhaseState::MovementPhaseState_unavailable:
                fprintf(stdout, "unavailable\n");
                break;
							case e_MovementPhaseState::MovementPhaseState_dark:
                fprintf(stdout, "dark\n");
                break;
							case e_MovementPhaseState::MovementPhaseState_stop_Then_Proceed:
                fprintf(stdout, "stop_Then_Proceeddark\n");
                break;
							case e_MovementPhaseState::MovementPhaseState_stop_And_Remain:
                fprintf(stdout, "stop_And_Remain\n");
                break;
							case e_MovementPhaseState::MovementPhaseState_pre_Movement:
                fprintf(stdout, "pre_Movement\n");
                break;
							case e_MovementPhaseState::MovementPhaseState_permissive_Movement_Allowed:
								fprintf(stdout, "permissive_Movement_Allowed\n");
                break;
							case e_MovementPhaseState::MovementPhaseState_protected_Movement_Allowed:
								fprintf(stdout, "protected_Movement_Allowed\n");
                break;
							case e_MovementPhaseState::MovementPhaseState_permissive_clearance:
								fprintf(stdout, "permissive_clearance\n");
                break;
							case e_MovementPhaseState::MovementPhaseState_protected_clearance:
								fprintf(stdout, "protected_clearance\n");
                break;
							case e_MovementPhaseState::MovementPhaseState_caution_Conflicting_Traffic:
								fprintf(stdout, "caution_Conflicting_Traffic\n");
                break;
							default:
								fprintf(stdout, "Impossible situation happened. please check this\n");
                exit(-1);
						}

            fprintf(stdout, "          timing\n");
            fprintf(stdout, "            minEndTime : %ld\n", ptr->states.list.array[j]->state_time_speed.list.array[k]->timing->minEndTime);
          }
        }
        // fprintf(stdout, "          regional           : %ld\n", ptr->states.list.array[j]->regional);
      
        if (ptr->states.list.array[j]->maneuverAssistList->list.count)
        {
          fprintf(stdout, "    maneuverAssistList : %ld items\n", ptr->states.list.array[j]->maneuverAssistList->list.count);
        
          for (int j = 0; j < ptr->states.list.array[j]->maneuverAssistList->list.count; ++j) 
          {
            fprintf(stdout, "      maneuverAssistList[%ld]\n", j);
            fprintf(stdout, "        connectionID     : %ld\n",  ptr->states.list.array[j]->maneuverAssistList->list.array[j]->connectionID);

            if (ptr->states.list.array[j]->maneuverAssistList->list.array[j]->pedBicycleDetect)
            {
              if (*ptr->states.list.array[j]->maneuverAssistList->list.array[j]->pedBicycleDetect == true)
              {
                fprintf(stdout, "        pedBicycleDetect : True\n");
              }
              else
              {
                fprintf(stdout, "        pedBicycleDetect : False\n");
              }
            }
          }
        }
      }
    }	
  }

  return 0;
}

/* TX_WAVE_UPER(_RESULT) */
void IchthusV2X::sendPvd()
{
  while(1)
  {
    if(done) 
    {
      close(sockFd);
      exit(-1);
    }
    
    if(tx_v2i_pvd() < 0)
    {
      std::cout << "DEBUG : disconnect TCP, retry" << endl;
      close(sockFd);
      exit(1);
    }

    // usleep(1000);
  }
}

int IchthusV2X::tx_v2i_pvd()
{
  unsigned long long interval = get_clock_time() - txPvd;  // msec;

  if(interval < PVD_INTERVAL)
  {
    return 0;
  }

  // 이 부분 뭘까...?  
  txPvd += (interval - interval%PVD_INTERVAL); 

  MessageFrame_t msg;
  char uper[MAX_UPER_SIZE]; 

  // 인코딩할 PVD 채우기
  fill_j2735_pvd(&msg);

  std::cout << ">> Parse J2735 : PVD <<" << endl;
  parse_pvd(&msg);
  gen_PvdMsg(&msg);

  // pvd_pub_->publish(pvd_msg); /* for test */

  // 인코딩
  int encodedBits = encode_j2735_uper(uper, &msg);

  //printf("bits:%d\n",encodedBits);
  if(encodedBits < 0) // 인코딩 실패로 전송이 불가능한 상태
  {
    return 0;
  }
    
  int byteLen = encodedBits / 8 + ((encodedBits % 8)? 1:0);
    
  //print_hex(uper,byteLen);    
      
  return request_tx_wave_obu(uper,byteLen);  
}

void IchthusV2X::print_hex(char *data, int len)
{
  std::cout << "HEX[" << len << "] : ";
  for(int i = 0 ; i < len ; i++)
  {
    printf("%02X",(data[i] & 0xFF));
  }
  printf("\n");
}

int IchthusV2X::fill_j2735_pvd(MessageFrame_t *dst)
{
  // 현재 날짜, 시간
  time_t timer;
  struct tm *t;
  timer = time(NULL);
  t = localtime(&timer);

  //ASN_STRUCT_RESET(asn_DEF_MessageFrame, dst);

  dst->messageId = 26; // J2735 표준문서 PDF 파일 참조 DE_DSRC_MessageID,  probeVehicleData DSRCmsgID ::= 26 -- PVD 
  dst->value.present = MessageFrame__value_PR_ProbeVehicleData; // MessageFrame::value choice (asn1c)

  ProbeVehicleData_t *ptrPvd = &dst->value.choice.ProbeVehicleData;

  ptrPvd->timeStamp = NULL; // OPTIONAL, not to use
  ptrPvd->segNum = NULL;    // OPTIONAL, not to use
  ptrPvd->regional = NULL;  // OPTIONAL, not to use

  // ptrPvd->probeID = malloc(sizeof(struct VehicleIdent));
  ptrPvd->probeID = new(struct VehicleIdent);
  ptrPvd->probeID->name = NULL;         // OPTIONAL, not to use
  ptrPvd->probeID->ownerCode = NULL;    // OPTIONAL, not to use
  ptrPvd->probeID->vehicleClass = NULL; // OPTIONAL, not to use
  ptrPvd->probeID->vin = NULL;          // OPTIONAL, not to use
  ptrPvd->probeID->vehicleType = NULL;  // OPTIONAL, not to use
  // ptrPvd->probeID->id = malloc(sizeof (struct VehicleID));
  ptrPvd->probeID->id = new(struct VehicleID);
  ptrPvd->probeID->id->present = VehicleID_PR_entityID;   
  ptrPvd->probeID->id->present = VehicleID_PR_entityID;
  ptrPvd->probeID->id->choice.entityID.buf = (unsigned char *)malloc(4);
  ptrPvd->probeID->id->choice.entityID.size = 4; 
  ptrPvd->probeID->id->choice.entityID.buf[0] = 0xCE;      // (INPUT) <---- 할당된 대학별 ID 입력
  ptrPvd->probeID->id->choice.entityID.buf[1] = 0x24;      // (INPUT) <---- 할당된 대학별 ID 입력
  ptrPvd->probeID->id->choice.entityID.buf[2] = 0x67;      // (INPUT) <---- 할당된 대학별 ID 입력
  ptrPvd->probeID->id->choice.entityID.buf[3] = 0x04;      // (INPUT) <---- 할당된 대학별 ID 입력

  //StartVector : PVD를 전송할 시점을 기준의 시간과 차량의 위치, 이동상태 값을 반영  
  // ptrPvd->startVector.utcTime = malloc(sizeof(struct DDateTime));  
  // ptrPvd->startVector.utcTime->year = malloc(sizeof(DYear_t));
  // ptrPvd->startVector.utcTime->month = malloc(sizeof(DMonth_t)); 
  // ptrPvd->startVector.utcTime->day = malloc(sizeof(DDay_t)); 
  // ptrPvd->startVector.utcTime->hour = malloc(sizeof(DHour_t)); 
  // ptrPvd->startVector.utcTime->minute = malloc(sizeof(DMinute_t)); 
  // ptrPvd->startVector.utcTime->second = malloc(sizeof(DSecond_t));
  ptrPvd->startVector.utcTime = new(struct DDateTime);  
  ptrPvd->startVector.utcTime->year = new DYear_t;
  ptrPvd->startVector.utcTime->month = new DMonth_t; 
  ptrPvd->startVector.utcTime->day = new DDay_t; 
  ptrPvd->startVector.utcTime->hour = new DHour_t; 
  ptrPvd->startVector.utcTime->minute = new DMinute_t; 
  ptrPvd->startVector.utcTime->second = new DSecond_t;  
  ptrPvd->startVector.utcTime->offset = NULL; // OPTIONAL, not to use

  *ptrPvd->startVector.utcTime->year = t->tm_year+1900; // (INPUT) <--------------- 현재 UTC 시간 입력 (년도)
  *ptrPvd->startVector.utcTime->month = t->tm_mon+1;   // (INPUT) <--------------- 현재 UTC 시간 입력 (년도)
  *ptrPvd->startVector.utcTime->day = t->tm_mday;     // (INPUT) <--------------- 현재 UTC 시간 입력 (년도)
  *ptrPvd->startVector.utcTime->hour = t->tm_hour;    // (INPUT) <--------------- 현재 UTC 시간 입력 (년도)
  *ptrPvd->startVector.utcTime->minute = t->tm_min;  // (INPUT) <--------------- 현재 UTC 시간 입력 (년도)
  *ptrPvd->startVector.utcTime->second = t->tm_sec;  // (INPUT) <--------------- 현재 UTC 시간 입력 (년도)

  // ptrPvd->startVector.elevation = malloc(sizeof(DSRC_Elevation_t));
  // ptrPvd->startVector.heading = malloc(sizeof(Heading_t));
  // ptrPvd->startVector.speed = malloc(sizeof(struct TransmissionAndSpeed));
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

  // ptrPvd->vehicleType.hpmsType = malloc(sizeof(VehicleType_t));
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

  // PVD 전송 직전에 전송한 PVD startVector 시간, 위치, 이동상태를 입력 
  ptrPvd->snapshots.list.count = 1; 
  // ptrPvd->snapshots.list.array = malloc(sizeof(struct Snapshot *));
  // ptrPvd->snapshots.list.array[0] = malloc(sizeof(struct Snapshot));
  ptrPvd->snapshots.list.array = new (struct Snapshot *);
  ptrPvd->snapshots.list.array[0] = new (struct Snapshot);
  struct Snapshot *ptrSnapshot = ptrPvd->snapshots.list.array[0]; 

  // ptrSnapshot->thePosition.utcTime = malloc(sizeof(struct DDateTime));
  // ptrSnapshot->thePosition.utcTime->year = malloc(sizeof(DYear_t));
  // ptrSnapshot->thePosition.utcTime->month = malloc(sizeof(DMonth_t));
  // ptrSnapshot->thePosition.utcTime->day = malloc(sizeof(DDay_t));
  // ptrSnapshot->thePosition.utcTime->hour = malloc(sizeof(DHour_t));
  // ptrSnapshot->thePosition.utcTime->minute = malloc(sizeof(DMinute_t));
  // ptrSnapshot->thePosition.utcTime->second = malloc(sizeof(DSecond_t));
  ptrSnapshot->thePosition.utcTime = new (struct DDateTime);
  ptrSnapshot->thePosition.utcTime->year = new DYear_t;
  ptrSnapshot->thePosition.utcTime->month = new DMonth_t;
  ptrSnapshot->thePosition.utcTime->day = new DDay_t;
  ptrSnapshot->thePosition.utcTime->hour = new DHour_t;
  ptrSnapshot->thePosition.utcTime->minute = new DMinute_t;
  ptrSnapshot->thePosition.utcTime->second = new DSecond_t;
  ptrSnapshot->thePosition.utcTime->offset = NULL; // OPTIONAL, not to use

  // ptrSnapshot->thePosition.elevation = malloc(sizeof(DSRC_Elevation_t));
  // ptrSnapshot->thePosition.speed = malloc(sizeof(struct TransmissionAndSpeed));
  // ptrSnapshot->thePosition.heading = malloc(sizeof(Heading_t));
  ptrSnapshot->thePosition.elevation = new DSRC_Elevation_t;
  ptrSnapshot->thePosition.speed = new (struct TransmissionAndSpeed);
  ptrSnapshot->thePosition.heading = new Heading_t;
  ptrSnapshot->thePosition.posAccuracy = NULL;     // OPTIONAL, not to use
  ptrSnapshot->thePosition.posConfidence = NULL;   // OPTIONAL, not to use
  ptrSnapshot->thePosition.timeConfidence = NULL;  // OPTIONAL, not to use
  ptrSnapshot->thePosition.speedConfidence = NULL; // OPTIONAL, not to use

  *ptrSnapshot->thePosition.utcTime->year = t->tm_year+1900;    // (INPUT) <--------------- 직전 전송한 PVD의 UTC 시간 입력 (년도)
  *ptrSnapshot->thePosition.utcTime->month = t->tm_mon+1;       // (INPUT) <--------------- 직전 전송한 PVD의 UTC 시간 입력 (월)
  *ptrSnapshot->thePosition.utcTime->day = t->tm_mday;          // (INPUT) <--------------- 직전 전송한 PVD의 UTC 시간 입력 (일)
  *ptrSnapshot->thePosition.utcTime->hour = t->tm_hour;         // (INPUT) <--------------- 직전 전송한 PVD의 UTC 시간 입력 (시)
  *ptrSnapshot->thePosition.utcTime->minute = t->tm_min;        // (INPUT) <--------------- 직전 전송한 PVD의 UTC 시간 입력 (분)
  *ptrSnapshot->thePosition.utcTime->second = t->tm_sec;        // (INPUT) <--------------- 직전 전송한 PVD의 UTC 시간 입력 (초)
  
  ptrSnapshot->thePosition.lat = Vli_.latitude;                   // (INPUT) <--------------- 현재 차량의 위치 (위도) (Longitude, DD 좌표계)
  ptrSnapshot->thePosition.Long = Vli_.longitude;                 // (INPUT) <--------------- 현재 차량의 위치 (경도) (Latitude,  DD 좌표계) 
  *ptrSnapshot->thePosition.elevation = Vli_.elevation;           // (INPUT) <--------------- 현재 차량의 위치 (고도) (Elevation)   
  *ptrSnapshot->thePosition.heading = Vli_.heading;               // (INPUT) <--------------- 현재 차량의 주행 방향 (북쪽 0도)               
  ptrSnapshot->thePosition.speed->speed = Vli_.speed;             // (INPUT) <-------------- -현재 차량의 속도                  
  ptrSnapshot->thePosition.speed->transmisson = Vli_.transmisson; // (INPUT) <--------------- 현재 차량의 변속기 상태          

  return 0;
}

void IchthusV2X::gen_PvdMsg(MessageFrame_t *pvd)
{

  pvd_msg.msg_type = 5;

}

int IchthusV2X::parse_pvd(MessageFrame_t *pvd)
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
    printf("    transmisson : %ld\n", ptrSnapshot->thePosition.speed->transmisson);
  }

  return 0;
}

int IchthusV2X::encode_j2735_uper(char *dst, MessageFrame_t *src)
{
  int res = -1;

  asn_enc_rval_t ret = uper_encode_to_buffer(&asn_DEF_MessageFrame,
                                              NULL,
                                              src,
                                              dst, MAX_UPER_SIZE);
    
  if (ret.encoded > 0)
  {
    return ret.encoded; //  UPER Encoding Success
  }
  else
  { 
    if (ret.failed_type != NULL)
    {
      std::cout << "encoded error value name = " << ret.failed_type->name << std::endl;
    }
    return -1; // UPER Encoding failed
  }
}

int IchthusV2X::request_tx_wave_obu(char *uper, unsigned short uperLength)
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

  int res = write(sockFd, packet, packetLen);

  if (res > 0)
  {
    printf("TX - \"TX_WAVE_UPER\" SEQ[%d] = ", ptrHeader->seq);
    print_hex(uper, uperLength);

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
}

void IchthusV2X::sigint_handler(int sig)
{  
	fprintf(stdout,"done !\n");
	done = true;  
} 

void IchthusV2X::install_signal_handler(void)
{
  struct sigaction sa; 

  memset(&sa, 0, sizeof sa); 
  sa.sa_handler = sigint_handler;

  if (sigaction(SIGINT, &sa, NULL) < 0) {
    fprintf(stderr,"ctrl+c sigint install failed \n");
    exit(-1);
  }
  fprintf(stdout,"Press <Ctrl-C> to exit\n");
}

}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ichthus_v2x::IchthusV2X>();
  
  // rclcpp::spin(node);
  rclcpp::Rate rate(5);
  while(rclcpp::ok())
  {
    rclcpp::spin_some(node);
    rate.sleep();
  }

  return 0;
}