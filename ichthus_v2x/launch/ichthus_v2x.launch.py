# ===========================================================================
# Copyright 2022. The ICHTHUS Project. All Rights Reserved.
# Sumin In (ism0705@naver.com),
# Youngjun Han (young@ssu.ac.kr)
# Vision System Laboratory, Soongsil University.
# added by ICHTHUS, Sumin In on 20221026
# [Licensed under the MIT License]  
# ===========================================================================

import os
from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
import yaml
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    
    ichthus_v2x = Node(
            package='ichthus_v2x',
            executable='ichthus_v2x',
            name='ichthus_v2x',
            parameters=[
                {
                    'ip' : LaunchConfiguration('ip'),
                    'debug' : LaunchConfiguration('debug'),
                    'display' : LaunchConfiguration('display')
                }
            ],
            # output='screen',
            output='log'
        )

    return [ichthus_v2x]


def generate_launch_description():

    launch_arguments = []

    def add_launch_arg(name: str, default_value=None):
        launch_arguments.append(DeclareLaunchArgument(name, default_value=default_value))

    add_launch_arg('ip', "192.168.10.10") # test_serv : 118.45.183.36 , OBU : 192.168.10.10
    add_launch_arg('debug', 'false')
    add_launch_arg('display', 'false')
    
    return launch.LaunchDescription(
        launch_arguments 
        + [OpaqueFunction(function=launch_setup)]
    )