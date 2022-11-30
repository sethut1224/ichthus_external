/*  ===========================================================================
    Copyright 2022. The ICHTHUS Project. All Rights Reserved.
    Sumin In (ism0705@naver.com),
    Youngjun Han (young@ssu.ac.kr)
    Vision System Laboratory, Soongsil University.
    added by ICHTHUS, Sumin In on 20221026
    [Licensed under the MIT License]  
    ===========================================================================*/

#pragma once

#ifndef __OBU_MSG_H__
#define __OBU_MSG_H__

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
#include <unistd.h> 
#include <time.h>
#include <arpa/inet.h>
#include <netinet/tcp.h>
#include <sys/socket.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "kiapi_msgs/msg/mission_list_stage1.hpp"
#include "kiapi_msgs/msg/mission_data.hpp"
#include "kiapi_msgs/msg/mission_route_data.hpp"
#include "kiapi_msgs/msg/mission_list_stage2.hpp"
#include "kiapi_msgs/msg/item_data.hpp"
#include "kiapi_msgs/msg/current_item.hpp"

#define DEVICE_ID_SIZE 3 
#define MSG_ENDPOINT 0x0D0A
#define MAX_BUF_LEN  4096 // max receive buffer size
#define MAX_BUFFER_LEN  1000000

#define PORT 24000

enum MessageType{

    MISSION_LIST_STAGE1 = 0x341A,
    MISSION_LIST_STAGE2 = 0x341B,
    AUTONOMOUS_REQUEST = 0x431C,
    AUTONOMOUS_RESPONSE = 0x341C,
    MISSION_STAGE_2_ITEM_GET = 0x341D,
    MISSION_STAGE_1_STATE_NODE = 0x341E

};
enum RequestType{

    REQ_SELECT_MISSION = 0x01,
    REQ_START_POSITION = 0x02,
    REQ_END_POSITION = 0x03,
    GET_ITEM = 0x04,
    PASS_NODE = 0x05,
};

enum ResponseType{
    
    RES_SUCCESS = 0x00

};


enum ItemType{

    ITEM_LAP_TIME_SUB    = 0x01,
    ITEM_LAP_TIME_ADD   = 0x02,
    ITEM_BOOST    = 0x03,
    
};

#pragma pack(1)
struct MsgHeader{

    uint16_t message_type;
    uint8_t sequence;
    uint16_t payload_length;
    uint8_t device_type;
    uint8_t device_id[DEVICE_ID_SIZE];

};

#pragma pack(1)
struct MissionData{
    
    uint8_t mission_id;
    uint8_t status;
    uint8_t event_count;
    uint16_t distance;
    uint8_t route_id;
    int32_t start_lat;
    int32_t start_Lon;
    int32_t end_lat;
    int32_t end_lon;
    
};

#pragma pack(1)
struct MissionRouteData{
    
    uint8_t mission_route_id;
    uint8_t route_node_total_count;
    uint8_t route_node_index;
    uint8_t route_node_type;
    int32_t route_node_pos_lat;
    int32_t route_node_pos_lon;

};

#pragma pack(1)
struct MissionListStage1{
    
    MsgHeader header;
    uint8_t mission_status;
    uint8_t mission_count;
    uint16_t mission_route_count;
    
    MissionData *mission_list;
    MissionRouteData *mission_route_list;
    
};

#pragma pack(1)
struct ItemData{
    
    uint8_t item_id;
    uint8_t item_type;
    uint8_t item_status;
    int32_t score;
    uint8_t speed;
    uint8_t duration;
    int32_t pos_lat;
    int32_t pos_long;
    uint8_t extend;

};

#pragma pack(1)
struct MissionListStage2{
    
    MsgHeader header;
    uint8_t mission_status;
    uint8_t item_count;
    
    ItemData *item_list;
    
};

#pragma pack(1)
struct Request{
    
    MsgHeader header;
    uint8_t mission_id;
    uint8_t request;
    uint8_t response;
    char description[10];
    char temporary[20];
    uint16_t end_point;

};

#pragma pack(1)
struct Request_Ack{
    
    MsgHeader header;
    uint8_t mission_id;
    uint8_t request;
    uint8_t response;
    char description[10];
    char temporary[20];
    uint16_t end_point;

};

#pragma pack(1)
struct Item_Ack{
    
    MsgHeader header;
    uint8_t item_id;
    uint8_t request;
    uint8_t response;
    char description[10];
    char temporary[20];
    uint16_t end_point;

};
#endif //__OBU_MSG_H__