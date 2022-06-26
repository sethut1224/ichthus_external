# ichthus_v2x

V2X ROS2 node of Soongsil University, Ichthus


## Process
 1. Receive J2735(SPaT) Messages  
 Recevie SPaT(Traffic light Information) Messages through WAVE socket communication and publish "/v2x_info" ros2 topic  
 
 2. Send J2735(PVD) Messages  
 Subscribe other ros node topic, fill the correct PVD message and send PVD message through WAVE socket communication  

## Requirements
 1. Information of gnss
 2. Information of Can data
 3. OBU(CEST)
 4. ASN.1 Compiler
 5. J2735 ASN file
 
## ROS API
#### Subs
* ```/fix``` ([sensor_msgs/msg/NavSatFix](http://docs.ros.org/en/lunar/api/sensor_msgs/html/msg/NavSatFix.html))  
  Information of gnss - longitude, latitude, elevation
* ```/v2x``` ([std_msgs/msg/Float64MultiArray](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float64MultiArray.html))  
  Information of Can data - speed, transmission
* ```/gnss_pose``` ([geometry_msgs/msg/PoseStamped](http://docs.ros.org/en/lunar/api/geometry_msgs/html/msg/PoseStamped.html))  
  Information of gnss orientation - heading  

#### Pubs
* ```/v2x_info``` ([kiapi_msgs/msg/V2xinfo](https://github.com/simfrog/2022-kiapi_msgs/blob/main/msg/V2xinfo.msg))  
  Information of SPaT messages  
* ```/pvd_info``` ([kiapi_msgs/msg/Pvdinfo](https://github.com/simfrog/2022-kiapi_msgs/blob/main/msg/Pvdinfo.msg))  
  Information of PVD messages -  *DEBUG*

## Using ASN.1 Compiler to Use J2735
 1. Download asn1c source code  
 or bash command > git clone https://github.com/vlm/asn1c.git
 ![Download ASN 1](https://user-images.githubusercontent.com/31130917/174310844-a9cc2179-e125-4987-be76-0ef77ea3f434.png)  
 2. Build asn1c source code and install  
 * Build environment configuration  
   * apt-get update
   * apt-get install autoconf libtool gcc g++ make
 * Build asn1c source code
   * cd [asn1c source code dir]
   * autoreconf -iv
   * ./configure
   * make install
   * asn1c -v  
![asn1c build](https://user-images.githubusercontent.com/31130917/174312820-2e08fbed-9de5-4a1a-9e4f-58cc92b611b6.png)  
 3. Compile ASN.1  
 * Copy ASN file within a asn folder
 * Execute compile commands within a j2735 folder
   * cd j2735
   * asn1c -f compound-names -pdu=all ../asn/[.ASN file]
   * make -f converter-example.mk
 * Check the creation of libasncodec.a within a j2735 folder  
![asn 1 compile](https://user-images.githubusercontent.com/31130917/174422636-c1b9bc90-a9aa-4896-9ec5-24baf529dd12.png)  

## How to launch
```ros2 launch ichthus_v2x ichthus_v2x.launch.py```
