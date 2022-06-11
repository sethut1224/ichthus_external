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
        self.use_imu = LaunchConfiguration('use_imu').perform(self.context)
        self.twist_topic = '/localization/twist_estimator/twist_with_covariance' if self.use_imu == 'true' or 'True' else '/localization/twist_estimator/vehicle_velocity_converter/twist_with_covariance'

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

    def localization_pipeline(self):
        
        ndt_scan_matcher_param_path = LaunchConfiguration('ndt_scan_matcher_param_path').perform(self.context)
        with open(ndt_scan_matcher_param_path, "r") as f:
            ndt_scan_matcher_param = yaml.safe_load(f)["/**"]["ros__parameters"]

        vehicle_velocity_converter_param_path = LaunchConfiguration('vehicle_velocity_converter_param_path').perform(self.context)
        with open(vehicle_velocity_converter_param_path, 'r') as f:
            vehicle_velocity_converter_param = yaml.safe_load(f)["/**"]["ros__parameters"]

        pose_initializer_param_path = LaunchConfiguration('pose_initializer_param_path').perform(self.context)
        with open(pose_initializer_param_path, 'r') as f:
            pose_initializer_param = yaml.safe_load(f)["/**"]["ros__parameters"]

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
                ('twist_with_covariance', '/localization/twist_estimator/vehicle_velocity_converter/twist_with_covariance')
            ]
        )

        gyro_odometer = Node(
            package='gyro_odometer',
            executable='gyro_odometer',
            name='gyro_odometer',
            remappings=[
                ('vehicle/twist_with_covariance', '/localization/twist_estimator/vehicle_velocity_converter/twist_with_covariance'),
                ('imu', '/imu/imu_data'),
                ('twist_raw','gyro_twist_raw'),
                ('twist_with_covariance_raw', '/localization/twist_estimator/twist_with_covariance_raw'),
                ('twist', 'gyro_twist'),
                ('twist_with_covariance', '/localization/twist_estimator/twist_with_covariance')
            ],
            parameters=[
                {
                    'output_frame' : 'base_link',
                    'message_timeout_sec' : 0.2,
                    'use_sim_time' : LaunchConfiguration('use_sim_time'),
                }
            ],
            condition=IfCondition(LaunchConfiguration('use_imu'))
        )

        pose_initializer = Node(
            package='pose_initializer',
            executable='pose_initializer',
            name='pose_initializer',
            remappings=[
                ('initialpose', '/initialpose'),
                ('initialpose3d', '/initialpose3d'),
                ('gnss_pose_cov', '/sensing/gnss/pose_with_covariance'),
                ('pointcloud_map', '/pointcloud_map'),
                ('ndt_align_srv', '/ndt_align_srv'),
                ('service/initialize_pose', '/localization/util/initialize_pose'),
                ('service/initialize_pose_auto', '/localization/util/initialize_pose_auto')
            ],
            parameters=[
                pose_initializer_param,
                {
                    'enable_gnss_callback' : False,
                    'use_sim_time' : LaunchConfiguration('use_sim_time'),
                }
            ]
        )

        voxel_grid_downsample_filter_param_path = LaunchConfiguration('voxel_grid_downsample_filter_param_path').perform(self.context)

        with open(voxel_grid_downsample_filter_param_path, "r") as f:
            voxel_grid_downsample_filter_param = yaml.safe_load(f)["/**"]["ros__parameters"]
            
        voxel_grid_downsample_filter = Node(
            package="pointcloud_preprocessor",
            executable='voxel_grid_downsample_filter_node',
            name="voxel_grid_downsample_filter",
            remappings=[
                ('input', LaunchConfiguration('raw_pointcloud_topic')),
                ("output", "/voxel_grid_downsample/pointcloud"),
            ],

            parameters=[
                voxel_grid_downsample_filter_param,
                {
                    'voxel_size_x' : 2.0,
                    'voxel_size_y' : 2.0,
                    'voxel_size_z' : 2.0,
                    'max_queue_size' : 1,
                    'use_sim_time' : LaunchConfiguration('use_sim_time'),
                }
            ],
        )

        ndt_scan_matcher = Node(
            package='ndt_scan_matcher',
            executable='ndt_scan_matcher',
            name='ndt_scan_matcher',
            parameters=[
                ndt_scan_matcher_param,
                {
                    'use_sim_time' : LaunchConfiguration('use_sim_time'),
                    'resolution' : 3.0,
                    'ndt_implement_type' : 2,  # 0=PCL_GENERIC, 1=PCL_MODIFIED, 2=OMP
                    'converged_param_type' : 1, # 0=TRANSFORM_PROBABILITY, 1=NEAREST_VOXEL_TRANSFORMATION_LIKELIHOOD
                                            # NEAREST_VOXEL_TRANSFORMATION_LIKELIHOOD is only available when NDTImplementType::OMP is selected
                }
            ],

            remappings=[
                ('points_raw', '/voxel_grid_downsample/pointcloud'),
                ('ekf_pose_with_covariance', '/localization/pose_twist_fusion_filter/pose_with_covariance'),
                ('pointcloud_map', '/pointcloud_map'),
                ('ndt_pose', '/localization/pose_estimator/pose'),
                ('ndt_pose_with_covariance','/localization/pose_estimator/pose_with_covariance')
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
                    'tf_rate' : 50.0,
                    'extend_state_step' : 50,
                    'pose_additional_delay' : 0.0,
                    'pose_measure_uncertainty_time' : 0.015,
                    'pose_rate' : 10.0,
                    'pose_gate_dist' : 100.0,
                    'twist_additional_delay' : 0.01,
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
                ('initialpose', 'initialpose3d'),
                ('in_pose_with_covariance', '/localization/pose_estimator/pose_with_covariance'),
                ('in_twist_with_covariance', self.twist_topic),
                ('ekf_odom', '/localization/pose_twist_fusion_filter/kinematic_state'),
                ('ekf_pose', '/localization/pose_twist_fusion_filter/pose'),
                ('ekf_pose_with_covariance', '/localization/pose_twist_fusion_filter/pose_with_covariance'),
                ('ekf_pose_without_yawbias', '/localization/pose_twist_fusion_filter/pose_without_yawbias'),
                ('ekf_pose_with_covariance_without_yawbias', '/localization/pose_twist_fusion_filter/pose_with_covariance_without_yawbias'),
                ('ekf_twist','/localization/pose_twist_fusion_filter/twist'),
                ('ekf_twist_with_covariance', '/localization/pose_twist_fusion_filter/twist_with_covariance')
            ]
        )

        stop_filter = Node(
            package='stop_filter',
            executable='stop_filter',
            name='stop_filter',
            remappings=[
                ('input/odom','/localization/pose_twist_fusion_filter/kinematic_state'),
                ('input_twist_with_covariance_name', '/localization/pose_twist_fusion_filter/twist_with_covariance'),
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

        return [vehicle_velocity_converter, gyro_odometer, pose_initializer, voxel_grid_downsample_filter, ekf_localizer, ndt_scan_matcher, stop_filter]

def launch_setup(context, *args, **kwargs):
    pipeline = Localization(context)

    nodes = list()
    localization_nodes = pipeline.localization_pipeline()
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

    pose_initializer_param_path_default = os.path.join(
        get_package_share_directory('ichthus_launch'), 'param/pose_initializer.param.yaml'
    )

    voxel_grid_downsample_filter_param_path_default = os.path.join(
        get_package_share_directory('ichthus_launch'), 'param/voxel_grid_filter.param.yaml'
    )


    add_launch_arg('raw_pointcloud_topic', '/merged_cloud'),
    add_launch_arg('vehicle_info_param_path', vehicle_info_param_path_default),
    add_launch_arg('voxel_grid_downsample_filter_param_path', voxel_grid_downsample_filter_param_path_default)
    add_launch_arg('use_sim_time', 'False')
    add_launch_arg('ndt_scan_matcher_param_path', ndt_scan_matcher_param_path_default)
    add_launch_arg('vehicle_velocity_converter_param_path', vehicle_velocity_converter_param_path_default)
    add_launch_arg('pose_initializer_param_path',pose_initializer_param_path_default)
    add_launch_arg('use_imu', 'False')

    return launch.LaunchDescription(
        launch_arguments 
        + [OpaqueFunction(function=launch_setup)]
    )