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


class Localization:
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

    def localization_pipeline(self, downsampled_pointcloud):
        
        ndt_scan_matcher_param_path = LaunchConfiguration('ndt_scan_matcher_param_path').perform(self.context)
        with open(ndt_scan_matcher_param_path, "r") as f:
            ndt_scan_matcher_param = yaml.safe_load(f)["/**"]["ros__parameters"]

        vehicle_velocity_converter_param_path = LaunchConfiguration('vehicle_velocity_converter_param_path').perform(self.context)
        with open(vehicle_velocity_converter_param_path, 'r') as f:
            vehicle_velocity_converter_param = yaml.safe_load(f)["/**"]["ros__parameters"]

        vehicle_velocity_converter = Node(
            package="vehicle_velocity_converter",
            executable='vehicle_velocity_converter',
            name='vehicle_velocity_converter',

            parameters=[
                vehicle_velocity_converter_param,
                {
                    'use_sim_time' : LaunchConfiguration('use_sim_time'),
                }
            ],
            remappings=[
                ('velocity_status', '/vehicle/status/velocity_status'),
                ('twist_with_covariance', '/vehicle_velocity_converter/twist_with_covariance')
            ]
        )

        ndt_scan_matcher = Node(
            package='ndt_scan_matcher',
            executable='ndt_scan_matcher',
            name='ndt_scan_matcher',
            parameters=[
                ndt_scan_matcher_param,
                {
                    'use_sim_time' : LaunchConfiguration('use_sim_time')
                }
            ],

            remappings=[
                ('points_raw', downsampled_pointcloud),
                ('ekf_pose_with_covariance', '/localization/pose_with_covariance'),
                ('pointcloud_map', '/pointcloud_map'),
                ('ndt_pose', '/ndt_pose'),
                ('ndt_pose_with_covariacne','/ndt_pose_with_covariance')
            ]
        )

        ekf_localizer = Node(
            package='ekf_localizer',
            executable='ekf_localizer',
            name='ekf_localizer',
            parameters=[
                {
                    'pose_frame_id' : 'map',
                    'show_debug_info' : False,
                    'enable_yaw_bias_estimation' : False,
                    'predict_frequency' : 100.0,
                    'tf_rate' : 100.0,
                    'extend_state_step' : 50,
                    'pose_additional_delay' : 0.0,
                    'pose_measure_uncertainty_time' : 0.01,
                    'pose_rate' : 10.0,
                    'pose_gate_dist' : 10000.0,
                    'twist_additional_delay' : 0.0,
                    'twist_rate' : 100.0,
                    'twist_gate_dist' : 10000.0,
                    'proc_stddev_vx_c' : 5.0,
                    'proc_stddev_wz_c' : 1.0,
                    'twist_stddev_wz' : 0.003,
                    'proc_stddev_yaw_c' : 0.005,
                    'proc_stddev_yaw_bias_c' : 0.001,
                    'use_sim_time' : LaunchConfiguration('use_sim_time'),
                }
            ],
            remappings=[
                ('in_pose_with_covariance', '/ndt_pose_with_covariance'),
                ('in_twist_with_covariance', '/vehicle_velocity_converter/twist_with_covariance'),
                ('ekf_odom', '/localization/ekf_localizer/kinematic_state'),
                ('ekf_pose', '/localization/pose'),
                ('ekf_pose_with_covariance', '/localization/pose_with_covariance'),
                ('ekf_pose_without_yawbias', '/localization/pose_without_yawbias'),
                ('ekf_pose_with_covariance_without_yawbias', '/localization/pose_with_covariance_without_yawbias'),
                ('ekf_twist','/localization/twist'),
                ('ekf_twist_with_covariance', '/localization/ekf_localizer/twist_with_covariance')
            ]
        )

        stop_filter = Node(
            package='stop_filter',
            executable='stop_filter',
            name='stop_filter',
            remappings=[
                ('input/odom','/localization/ekf_localizer/kinematic_state'),
                ('input_twist_with_covariance_name', '/localization/ekf_localizer/twist_with_covariance'),
                ('/output/odom', '/localization/kinematic_state')
            ],
            parameters=[
                {
                    'vx_threshold' : 0.01,
                    'wz_threshold' : 0.01,
                    'use_sim_time' : LaunchConfiguration('use_sim_time'),
                }
            ]
        )

        return [vehicle_velocity_converter, ekf_localizer, ndt_scan_matcher, stop_filter]
        #imu_corrector, vehicle_velocity_converter, gyro_odometer ekf_localizer

def launch_setup(context, *args, **kwargs):
    pipeline = Localization(context)

    nodes = list()

    downsampled_pointcloud = LaunchConfiguration('downsampled_pointcloud').perform(context)
    localization_nodes = pipeline.localization_pipeline(downsampled_pointcloud)
    nodes.extend(localization_nodes)

    return nodes

def generate_launch_description():

    launch_arguments = []

    def add_launch_arg(name: str, default_value=None):
        launch_arguments.append(DeclareLaunchArgument(name, default_value=default_value))

    vehicle_info_param_path_default = os.path.join(
        get_package_share_directory('ichthus_launch'), 'param/vehicle_info.param.yaml'
    )

    ndt_scan_matcher_param_path_default = os.path.join(
        get_package_share_directory('ichthus_launch'), 'param/ndt_scan_matcher.param.yaml'
    )

    vehicle_velocity_converter_param_path_default = os.path.join(
        get_package_share_directory('ichthus_launch'), 'param/vehicle_velocity_converter.param.yaml'
    )

    add_launch_arg('downsampled_pointcloud', 'voxel_grid_downsample/pointcloud'),
    add_launch_arg('vehicle_info_param_path', vehicle_info_param_path_default),
    add_launch_arg('use_sim_time', 'False')
    add_launch_arg('ndt_scan_matcher_param_path', ndt_scan_matcher_param_path_default)
    add_launch_arg('vehicle_velocity_converter_param_path', vehicle_velocity_converter_param_path_default)
    return launch.LaunchDescription(
        launch_arguments 
        + [OpaqueFunction(function=launch_setup)]
    )