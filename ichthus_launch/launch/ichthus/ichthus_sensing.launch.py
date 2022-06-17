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


class Sensing:
    def __init__(self, context):
        self.context = context
        self.vehicle_info = self.get_vehicle_info()

    def get_vehicle_info(self):
        path = LaunchConfiguration('vehicle_info_param_path').perform(self.context)
        with open(path, 'r') as f:
            p = yaml.safe_load(f)["/**"]["ros__parameters"]
            p["vehicle_length"] = p["front_overhang"] + p["wheel_base"] + p["rear_overhang"]
            p["vehicle_width"] = p["wheel_tread"] + p["left_overhang"] + p["right_overhang"]
            p["min_longitudinal_offset"] = -p["rear_overhang"]
            p["max_longitudinal_offset"] = p["front_overhang"] + p["wheel_base"]
            p["min_lateral_offset"] = -(p["wheel_tread"] / 2.0 + p["right_overhang"])
            p["max_lateral_offset"] = p["wheel_tread"] / 2.0 + p["left_overhang"]
            p["min_height_offset"] = 0.0
            p["max_height_offset"] = p["vehicle_height"] 
        return p


    def imu(self):
        
        imu_corrector_param_path = LaunchConfiguration('imu_corrector_param_path').perform(self.context)

        with open(imu_corrector_param_path, "r") as f:
            imu_corrector_param = yaml.safe_load(f)["/**"]["ros__parameters"]

        imu_corrector = Node(
            package='imu_corrector',
            executable='imu_corrector',
            name='imu_corrector',
            remappings=[
                ('input', '/imu/imu_raw'),
                ('output', '/imu/imu_data')
            ],
            parameters =[
                imu_corrector_param,
                {
                    'use_sim_time' : LaunchConfiguration('use_sim_time')
                }
            ],
            condition=IfCondition(LaunchConfiguration('use_imu'))
        )

        return [imu_corrector]
    
    def gnss(self):
        g2m_poser = Node(
            package='g2m_poser',
            executable='g2m_poser_exe',
            name='g2m_poser',
            parameters =[
                {
                    'use_sim_time' : LaunchConfiguration('use_sim_time'),
                    'use_tf_publish': LaunchConfiguration('use_tf_publish'),
                    'target_frame_id': LaunchConfiguration('target_frame_id'),
                    'origin_x': LaunchConfiguration('origin_x'),
                    'origin_y': LaunchConfiguration('origin_y'),
                    'origin_z': LaunchConfiguration('origin_z'),
                    'heading_source': LaunchConfiguration('heading_source'),
                    'pose_diff_for_heading': LaunchConfiguration('pose_diff_for_heading'),
                    'fix_topic': LaunchConfiguration('fix_topic'),
                    'imu_topic': LaunchConfiguration('imu_topic'),
                    'odom_topic': LaunchConfiguration('odom_topic'),
                    'pose_topic': LaunchConfiguration('pose_topic'),
                    'pose_cov_topic': LaunchConfiguration('pose_cov_topic'),
                },
            ],
            condition=IfCondition(LaunchConfiguration('use_gnss'))
        )

        return [g2m_poser]

def launch_setup(context, *args, **kwargs):
    pipeline = Sensing(context)

    nodes = []

    imu_nodes = pipeline.imu()
    gnss_nodes = pipeline.gnss()

    nodes.extend(imu_nodes)
    nodes.extend(gnss_nodes)

    return nodes

def generate_launch_description():

    launch_arguments = []

    def add_launch_arg(name: str, default_value=None):
        launch_arguments.append(DeclareLaunchArgument(name, default_value=default_value))

    vehicle_info_param_path_default = os.path.join(
        get_package_share_directory('ichthus_launch'), 'param/vehicle_info.param.yaml'
    )

    imu_corrector_param_path_default = os.path.join(
        get_package_share_directory('ichthus_launch'), 'param/imu_corrector.param.yaml'
    )
    
    add_launch_arg('vehicle_info_param_path', vehicle_info_param_path_default),
    add_launch_arg('use_sim_time', 'False')
    add_launch_arg('imu_corrector_param_path', imu_corrector_param_path_default)
    add_launch_arg('use_imu', 'False')

    ############
    ### gnss ###
    ############
    add_launch_arg("use_tf_publish", "false")
    add_launch_arg("target_frame_id", "gnss")

    ### origin of city ###
    add_launch_arg("origin_x", "'445815.539508'")
    add_launch_arg("origin_y", "'3944953.128090'")
    add_launch_arg("origin_z", "'48.640911'")
    ######################
    
    add_launch_arg("heading_source", "NONE") # NONE, IMU, ODOM
    add_launch_arg("pose_diff_for_heading", "0.4")
    add_launch_arg("fix_topic", "/fix")
    add_launch_arg("imu_topic", "/imu/data")
    add_launch_arg("odom_topic", "/output/velocity_report")
    add_launch_arg("pose_topic", "/gnss_pose")
    add_launch_arg("pose_cov_topic", "/gnss_pose_cov")
    add_launch_arg("use_gnss", "true")
    

    return launch.LaunchDescription(
        launch_arguments
        + [OpaqueFunction(function=launch_setup)]
    )