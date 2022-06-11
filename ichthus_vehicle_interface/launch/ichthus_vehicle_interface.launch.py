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


class Interface:
    def __init__(self, context):
        self.context = context
        
    def interface(self):
    
        ichthus_vehicle_interface = Node(
            package='ichthus_vehicle_interface',
            executable='ichthus_vehicle_interface_node',
            name='ichthus_vehicle_interface',
        )

        return [ichthus_vehicle_interface]

def launch_setup(context, *args, **kwargs):
    pipeline = Interface(context)

    nodes = list()
    interface_nodes = pipeline.interface()
    nodes.extend(interface_nodes)

    return nodes

def generate_launch_description():

    launch_arguments = []

    def add_launch_arg(name: str, default_value=None):
        launch_arguments.append(DeclareLaunchArgument(name, default_value=default_value))
 

    return launch.LaunchDescription(
        launch_arguments 
        + [OpaqueFunction(function=launch_setup)]
    )