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
                ('input', '/imu/data'),
                ('output', '/imu/corrected_data')
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
        xsens_driver = Node( # mtnode.py
            package='xsens_driver_ros2',
            namespace='',
            executable='mtnode.py',
            name='mtnode',
            parameters=[
                {'device': 'auto'},
                {'baudrate': 0},
                {'timeout': 0.002},
                {'initial_wait': 0.1},
                {'frame_id': 'imu'},
                {'frame_local': 'ENU'},
                {'no_rotation_duration': 0},
                {'angular_velocity_covariance_diagonal': [0.0004, 0.0004, 0.0004]},
                {'linear_acceleration_covariance_diagonal': [0.0004, 0.0004, 0.0004]},
                {'orientation_covariance_diagonal': [0.01745, 0.01745, 0.15708]},
            ],
            output='screen',
        )

        g2m_poser = Node(
            package='g2m_poser',
            executable='g2m_poser_exe',
            name='g2m_poser',
            parameters =[
                {
                    'use_sim_time' : LaunchConfiguration('use_sim_time'),
                    'use_tf_publish': LaunchConfiguration('use_tf_publish'),
                    'use_b2g_tf_listener' : LaunchConfiguration('use_b2g_tf_listener'),
                    'base_link_frame': LaunchConfiguration('base_link_frame'),
                    'origin_x': LaunchConfiguration('origin_x'),
                    'origin_y': LaunchConfiguration('origin_y'),
                    'origin_z': LaunchConfiguration('origin_z'),
                    'b2g_x' : LaunchConfiguration('b2g_x'),
                    'b2g_y' : LaunchConfiguration('b2g_y'),
                    'b2g_z' : LaunchConfiguration('b2g_z'),
                    'heading_source': LaunchConfiguration('heading_source'),
                    'max_buffer_size': LaunchConfiguration('max_buffer_size'),
                    'pose_diff_for_heading': LaunchConfiguration('pose_diff_for_heading'),
                    'fix_topic': LaunchConfiguration('fix_topic'),
                    'imu_topic': LaunchConfiguration('imu_topic'),
                    'odom_topic': LaunchConfiguration('odom_topic'),
                    'pose_topic': LaunchConfiguration('pose_topic'),
                    'pose_cov_topic': LaunchConfiguration('pose_cov_topic'),
                    # 'pose_cov_topic' : self.gnss_pose_cov_topic
                },
            ],
            condition=IfCondition(LaunchConfiguration('use_gnss'))
        )

        return [g2m_poser, xsens_driver]    

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
    add_launch_arg("use_b2g_tf_listener", "false")
    add_launch_arg("base_link_frame", "base_link")

    ### origin of city ###
    add_launch_arg("origin_x", "'445815.539508'")
    add_launch_arg("origin_y", "'3944953.128090'")
    add_launch_arg("origin_z", "'48.640911'")
    ######################
    
    add_launch_arg("b2g_x", "0.359")
    add_launch_arg("b2g_y", "0.0")
    add_launch_arg("b2g_z", "1.348")

    add_launch_arg("heading_source", "NONE") # NONE, IMU, ODOM
    add_launch_arg("max_buffer_size", "5")
    add_launch_arg("pose_diff_for_heading", "0.2")
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