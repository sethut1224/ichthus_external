/*  ===========================================================================
    Copyright 2022. The ICHTHUS Project. All Rights Reserved.
    Sumin In (ism0705@naver.com),
    Youngjun Han (young@ssu.ac.kr)
    Vision System Laboratory, Soongsil University.
    added by ICHTHUS, Sumin In on 20221026
    [Licensed under the MIT License]  
    ===========================================================================*/

#include "ichthus_v2x/node.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ichthus_v2x::IchthusV2X>();
  
  rclcpp::Rate rate(20);
  while(rclcpp::ok())
  {
    rclcpp::spin_some(node);
    rate.sleep();
  }

  return 0;
}