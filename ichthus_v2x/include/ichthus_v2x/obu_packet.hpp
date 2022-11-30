/*  ===========================================================================
    Copyright 2022. The ICHTHUS Project. All Rights Reserved.
    Sumin In (ism0705@naver.com),
    Youngjun Han (young@ssu.ac.kr)
    Vision System Laboratory, Soongsil University.
    added by ICHTHUS, Sumin In on 20221026
    [Licensed under the MIT License]  
    ===========================================================================*/

#pragma once

#include <iostream>
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
#include <autoware_auto_perception_msgs/msg/traffic_signal_array.hpp>
 
#define OBU_RECEIVE_BUFFER_SIZE 2048 //받는 buffer의 전체 크기
#define MAX_UPER_SIZE 1400 //buffer 중 UPER(Payload) 크기

#define PVD_INTERVAL 100 //msec(5hz)

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