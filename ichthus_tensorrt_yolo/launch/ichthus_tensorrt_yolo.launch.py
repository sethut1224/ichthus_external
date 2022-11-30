import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml


def launch_setup(context, *args, **kwargs):
    trt_yolo_normal_info_param_path = os.path.join("/workspaces/isaac_ros-dev/src/ichthus_tensorrt_yolo/config/right_yolov4-csp.param.yaml")
    with open(trt_yolo_normal_info_param_path, "r") as f:
        trt_yolo_normal_info_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    trt_yolo_speed_bust_info_param_path = os.path.join("/workspaces/isaac_ros-dev/src/ichthus_tensorrt_yolo/config/right_yolov7_tiny_speed_bust.param.yaml")
    with open(trt_yolo_speed_bust_info_param_path, "r") as f:
        trt_yolo_speed_bust_info_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    yolo_normal_component = ComposableNode(
        package="ichthus_tensorrt_yolo",
        plugin="object_recognition::IchthusTensorrtYoloNodelet",
        name = "right_ichthus_trt_yolo_normal",
        namespace="",
        remappings=[],
        parameters=[trt_yolo_normal_info_param],
        extra_arguments=[{"use_intra_process_comms": True}],
    
    )

    yolo_speed_bust_component = ComposableNode(
        package="ichthus_tensorrt_yolo",
        plugin="object_recognition::IchthusTensorrtYoloNodelet",
        name = "right_ichthus_trt_yolo_speed_bust",
        namespace="",
        remappings=[],
        parameters=[trt_yolo_speed_bust_info_param],
        extra_arguments=[{"use_intra_process_comms": True}],
    )


    container=ComposableNodeContainer(
        name="right_ichthus_tensorrt_yolo_container",
        namespace="",
        package="rclcpp_components",
        executable=LaunchConfiguration("container_executable"),        
        composable_node_descriptions=[
            yolo_normal_component,
            yolo_speed_bust_component,
        ],
        output="screen",
    )

    group = GroupAction(
        [
            container
        ]
    )

    return [group]

def generate_launch_description():
    # component
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None, description=None):
        launch_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value, description=description)
    )

    add_launch_arg("use_intra_process", "false", "use ROS2 component container communication")
    add_launch_arg("use_multithread", "false", "use multithread")

    set_container_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container",
        condition=UnlessCondition(LaunchConfiguration("use_multithread")),
    )
    
    set_container_mt_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container_mt",
        # condition=IfCondition(LaunchConfiguration("use_multithread")),
    )

    return launch.LaunchDescription(
      launch_arguments
        +[
            set_container_executable
            # set_container_mt_executable
        ]+
        [OpaqueFunction(function=launch_setup)]
        +[
        Node(
            package='isaac_ros_argus_camera_mono',
            executable='isaac_ros_argus_camera_mono',
            parameters=[{
                'sensor': 0,
                'device': 0,
                'output_encoding': 'rgb8',
                'width': 960,
                'height': 640,
                'rate': 10 # hz
            }]
            )
        ]
    )
