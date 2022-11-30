# lte_mission

V2X(LTE) ROS2 node of Soongsil University, Ichthus


## Requirements
 1. Mission Manager
 2. OBU(CEST)

## ROS API
#### Subs
* ```/v2x/selected_mission``` ([std_msgs/msg/UInt8](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/UInt8.html))  
  The mission ID chosen by the mission manager
* ```/v2x/starting_point_arrive``` ([std_msgs/msg/Bool](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html))  
  Arriving at starting point completed
* ```/v2x/goal_arrive``` ([std_msgs/msg/Bool](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html))  
  Arriving at goal completed  

#### Pubs
* ```/v2x/stage1_mission_list``` ([kiapi_msgs/msg/MissionListStage1](https://github.com/simfrog/kiapi_msgs/blob/main/msg/MissionListStage1.msg))  
  Information of stage1 mission list
* ```/v2x/stage2_mission_list``` ([kiapi_msgs/msg/MissionListStage2](https://github.com/simfrog/kiapi_msgs/blob/main/msg/MissionListStage2.msg))  
  Information of stage2 mission list
* ```/v2x/current_item``` ([kiapi_msgs/msg/CurrentItem](https://github.com/simfrog/kiapi_msgs/blob/main/msg/CurrentItem.msg))  
  Information of items currently acquired
* ```/v2x/preempted_mission``` ([std_msgs/msg/UInt8](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/UInt8.html))  
  The preempted mission ID
* ```/v2x/error``` ([std_msgs/msg/UInt8](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/UInt8.html))  
  Information of error
* ```/v2x/passed_node``` ([std_msgs/msg/UInt8](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/UInt8.html))  
  Passed node index
* ```/v2x/arrive_response``` ([std_msgs/msg/UInt8](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/UInt8.html))  
  Responding to a request for completion of arrival at the starting point or goal  
  0 : error or doesn't arrive yet  
  1 : starting point arrive response  
  2 : goal arrvie response

## Results
#### Mission List Stage1 Exmaple
![route_data_ex](https://user-images.githubusercontent.com/31130917/191944011-535c965f-8cec-449e-b647-cee6951cceec.png)

#### Mission List Stage2 Exmaple
![item_ex](https://user-images.githubusercontent.com/31130917/191945436-edc03ffb-374b-414d-986e-db8e7eb60492.png)

## How to launch
```ros2 launch lte_mission lte_mission.launch.py```
