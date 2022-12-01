#!/bin/bash
# ===========================================================================
# Copyright 2022. The ICHTHUS Project. All Rights Reserved.
# Eunseo Choi (eunseo.choi.d@gmail.com) and Kanghee Kim (kim.kanghee@gmail.com).
# Mobility Intelligence & Computing Systems Laboratory, Soongsil University.
# ===========================================================================

if [ $# -ne 6 ] ; then
   echo "Usage: ./pub_item_index.sh [item_index]"
   exit 0
fi

ITEM_INDEX_0=$1
ITEM_INDEX_1=$2
ITEM_INDEX_2=$3
ITEM_INDEX_3=$4
ITEM_INDEX_4=$5
ITEM_INDEX_5=$6

ros2 topic pub /item_index std_msgs/msg/Int32 "data: 0" -1
ros2 topic pub /item_index std_msgs/msg/Int32 "data: 1" -1
ros2 topic pub /item_index std_msgs/msg/Int32 "data: 8" -1
ros2 topic pub /item_index std_msgs/msg/Int32 "data: 9" -1
ros2 topic pub /item_index std_msgs/msg/Int32 "data: 2" -1
ros2 topic pub /item_index std_msgs/msg/Int32 "data: 3" -1

ros2 topic pub /item_index std_msgs/msg/Int32 "data: $ITEM_INDEX_0" -1
ros2 topic pub /item_index std_msgs/msg/Int32 "data: $ITEM_INDEX_1" -1
ros2 topic pub /item_index std_msgs/msg/Int32 "data: $ITEM_INDEX_2" -1
ros2 topic pub /item_index std_msgs/msg/Int32 "data: $ITEM_INDEX_3" -1
ros2 topic pub /item_index std_msgs/msg/Int32 "data: $ITEM_INDEX_4" -1
ros2 topic pub /item_index std_msgs/msg/Int32 "data: $ITEM_INDEX_5" -1

ros2 topic pub /item_index std_msgs/msg/Int32 "data: 12" -1
