#!/bin/bash

if [ $# -ne 1 ] ; then
   echo "Usage: ./pub_file_name.sh [file_name]"
   exit 0
fi

FILE_NAME=$1

ros2 topic pub /save_mission_list std_msgs/msg/String "data: '$FILE_NAME'" -1

