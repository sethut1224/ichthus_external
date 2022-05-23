# Copyright 2021 the Autoware Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from distutils.sysconfig import get_config_h_filename
from email.policy import default
from re import M
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.conditions import LaunchConfigurationEquals
from launch.conditions import UnlessCondition
from launch.actions import ExecuteProcess

import os
import sys
import yaml
# sys.path.append('/root/autoware/src/launch/ichthus_launch/launch')
# import tcl_config


def generate_launch_description():

    launch_arguments = []
    def add_launch_arg(name: str, default_value=None):
        launch_arguments.append(DeclareLaunchArgument(name, default_value=default_value))

    perception = LaunchConfiguration('perception')
    planning = LaunchConfiguration('planning')
    control = LaunchConfiguration('control')
    lidar_channel = LaunchConfiguration('lidar_channel')
    lidar_detection_model = LaunchConfiguration('lidar_detection_model')

    add_launch_arg('perception', 'True')
    add_launch_arg('planning', 'False')
    add_launch_arg('control', 'False')
    add_launch_arg('lidar_channel', '16')
    add_launch_arg('lidar_detection_model', 'apollo')


    ichthus_launch_pkg_prefix = get_package_share_directory(
        'ichthus_launch')

    lgsvl_interface_node_param_file = os.path.join(
        ichthus_launch_pkg_prefix, 'param/lgsvl_interface.param.yaml' )

    lgsvl_interface_node_param = DeclareLaunchArgument(
        'lgsvl_interface_node_param_file',
        default_value=lgsvl_interface_node_param_file,
        description='Path to config file for Map Publisher Nodes' )

    lgsvl_interface = Node(
        package='lgsvl_interface',
        executable='lgsvl_interface_exe',
        name='lgsvl_interface',
        output='screen',
        parameters=[
          LaunchConfiguration('lgsvl_interface_node_param_file'),
          {"lgsvl.publish_tf": True},
          {"lgsvl.publish_pose": False}
        ],
        remappings=[
            ("vehicle_control_cmd", "/lgsvl/vehicle_control_cmd"),
            ('vehicle_command', '/vehicle/vehicle_command'),
            ("vehicle_state_cmd", "/lgsvl/vehicle_state_cmd"),
            ("state_report", "/lgsvl/state_report"),
            ("state_report_out", "/vehicle/state_report"),
            ("gnss_odom", "/lgsvl/gnss_odom"),
            ("vehicle_odom", "/lgsvl/vehicle_odom"),
            ('vehicle_kinematic_state', '/vehicle/vehicle_kinematic_state'),
            ('gear_report', '/vehicle/gear_report'),
            ('ackermann_vehicle_command', '/control/trajectory_follower/control_cmd')
        ]
    )

    
    urdf_path = os.path.join(ichthus_launch_pkg_prefix, 'urdf/lexus_rx_450h.urdf')
    with open(urdf_path, 'r') as infp:
        urdf_file = infp.read()

    vehicle_info_param_path = os.path.join(
        get_package_share_directory('ichthus_launch'), 'param/vehicle_info.param.yaml'
    )
    
    with open(vehicle_info_param_path, "r") as f:
        vehicle_info_param = yaml.safe_load(f)["/**"]["ros__parameters"]
    
    vehicle_info_param["vehicle_length"] = vehicle_info_param["front_overhang"] + vehicle_info_param["wheel_base"] + vehicle_info_param["rear_overhang"]
    vehicle_info_param["vehicle_width"] = vehicle_info_param["wheel_tread"] + vehicle_info_param["left_overhang"] + vehicle_info_param["right_overhang"]
    vehicle_info_param["min_longitudinal_offset"] = -vehicle_info_param["rear_overhang"]
    vehicle_info_param["max_longitudinal_offset"] = vehicle_info_param["front_overhang"] + vehicle_info_param["wheel_base"]
    vehicle_info_param["min_lateral_offset"] = -(vehicle_info_param["wheel_tread"] / 2.0 + vehicle_info_param["right_overhang"])
    vehicle_info_param["max_lateral_offset"] = vehicle_info_param["wheel_tread"] / 2.0 + vehicle_info_param["left_overhang"]
    vehicle_info_param["min_height_offset"] = 0.0
    vehicle_info_param["max_height_offset"] = vehicle_info_param["vehicle_height"]


    urdf_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='urdf_publisher',
        parameters=[
            {'robot_description': urdf_file}
        ],
        output='screen'
    )

    image_transport_decompressor = Node(
        package='image_transport_decompressor',
        executable='image_transport_decompressor_node',
        name='image_decompressor',
        parameters=[
            {"encoding": "rgb8"}
        ],
        remappings=[
            ('~/input/compressed_image', '/image/compressed'),
            ('~/output/raw_image', '/image_raw')
        ],
        condition=IfCondition(LaunchConfiguration('perception'))
    )

    tensorrt_yolo_param_path = os.path.join(
        ichthus_launch_pkg_prefix, 'param/tensorrt_yolo.param.yaml' )
    
    with open(tensorrt_yolo_param_path, "r") as f:
        tensorrt_yolo_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    tensorrt_yolo = Node(
        package='tensorrt_yolo',
        executable='tensorrt_yolo_node',
        name='tensorrt_yolo',
        parameters=[
            tensorrt_yolo_param
        ],
        remappings=[
            ('/in/image', '/image_raw'),
            ('/out/objects', '/camera/detected_objects')
        ],
        condition=IfCondition(LaunchConfiguration('perception'))
    )

    lanelet2_map_origin_path = os.path.join(
        get_package_share_directory("ichthus_launch"), "param/lanelet2_map_loader.param.yaml")

    with open(lanelet2_map_origin_path, "r") as f:
        lanelet2_map_origin_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    lanelet2_map_path = os.path.join(
        get_package_share_directory('ichthus_launch'), "map/SanFrancisco.osm" )

    lanelet2_map_path_param = DeclareLaunchArgument(
        'lanelet2_map_path',
        default_value=lanelet2_map_path )

    lanelet2_map_loader = Node(
        package="map_loader",
        executable='lanelet2_map_loader',
        name="lanelet2_map_loader",
        remappings=[("output/lanelet2_map", "/map/vector_map")],
        parameters=[
            {
                "center_line_resolution": 5.0,
                "lanelet2_map_path": LaunchConfiguration("lanelet2_map_path"),
                "lanelet2_map_projector_type": "UTM",  # Options: MGRS, UTM
            },
            lanelet2_map_origin_param,
        ],
    )

    lanelet2_map_visualization = Node(
        package="map_loader",
        executable='lanelet2_map_visualization',
        name="lanelet2_map_visualization",
        remappings=[
            ("input/lanelet2_map", "/map/vector_map"),
            ("output/lanelet2_map_marker", "vector_map_marker"),
        ],
    )
    
    map_odom_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=["0", "0", "10.578049659729004", "0", "0", "0", "map", "odom"] 
    )###only use when LGSVL Simulation without pointcloud map
     ###if Simulate only with lanelet2 map, fix the z value from the LGSVL 
     ###z is decide by the /lgsvl/gnss_odom z value
     ###if /lgsvl/gnss_odom x: 100, y:100, z:10.578049659729004, the z value of arguments is 10.578049659729004

    # pointcloud_map_path = '/root/shared_dir/autoware-data/SanFrancisco/data/map/pointcloud_map/SanFrancisco.pcd'

    # pointcloud_map_path_param = DeclareLaunchArgument(
    #     'pointcloud_map_path',
    #     default_value=pointcloud_map_path
    # )

    # pointcloud_map_loader = Node(
    #     package="map_loader",
    #     executable="pointcloud_map_loader",
    #     name="pointcloud_map_loader",
    #     remappings=[("output/pointcloud_map", "pointcloud_map")],
    #     parameters=[
    #         {"pcd_paths_or_directory": ["[", LaunchConfiguration("pointcloud_map_path"), "]"]}
    #     ],
    # )

    # map_hash_generator = Node(
    #     package="map_loader",
    #     executable="map_hash_generator",
    #     name="map_hash_generator",
    #     parameters=[
    #         {
    #             "lanelet2_map_path": LaunchConfiguration("lanelet2_map_path"),
    #             "pointcloud_map_path": LaunchConfiguration("pointcloud_map_path"),
    #         }
    #     ],
    # )

    # viewer_map_publisher = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments=["0", "0", "0", "0", "0", "0", "viewer", "map"]
    # )

    # map_tf_generator = Node(
    #     package="map_tf_generator",
    #     executable="map_tf_generator",
    #     name="map_tf_generator",
    #     parameters=[
    #         {
    #             "map_frame": "map",
    #             "viewer_frame": "viewer",
    #         }
    #     ],
    # )

    # voxel_grid_param_path = os.path.join(
    #     get_package_share_directory('ichthus_launch'), 'param/voxel_grid.param.yaml'
    # )
    # with open(voxel_grid_param_path, "r") as f:
    #     voxel_grid_param = yaml.safe_load(f)["/**"]["ros__parameters"]
    

    # approximate_downsample_filter = Node(
    #     package="pointcloud_preprocessor",
    #     executable="approximate_downsample_filter_node",
    #     name='approximate_downsample_filter',
    #     remappings=[
    #         ("input", '/lidar_front/points_raw'),
    #         ("output", "/lidar_front/downsampled/pointcloud"),
    #     ],
    #     parameters=[
    #         voxel_grid_param
    #     ],
    # )

    # outlier_param_path = os.path.join(
    #     get_package_share_directory('ichthus_launch'), 'param/outlier.param.yaml'
    # )
    # with open(outlier_param_path, "r") as f:
    #     outlier_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    # outlier_filter = Node(
    #     package="pointcloud_preprocessor",
    #     executable="voxel_grid_outlier_filter_node",
    #     name="outlier_filter",
    #     remappings=[
    #         ("input", "/lidar_front/downsampled/pointcloud"),
    #         ("output", "/lidar_front/outlier_filter/pointcloud")
    #     ],
    #     parameters=[outlier_param],
    # )

    crop_box_filter= Node(
        package='pointcloud_preprocessor',
        executable="crop_box_filter_node",
        name="crop_box_filter",
        remappings=[
            ("input",['/lidar_front/points_raw']),
            ('output', '/lidar_front/range_cropped_pointcloud')
        ],
        parameters=[
            {
                "input_frame": 'base_link',
                "output_frame": 'base_link',
                'min_x' : -60.0,
                'max_x' : 100.0,
                'min_y' : -60.0,
                'max_y' : 60.0,
                "min_z": -0.5,
                "max_z": vehicle_info_param['max_height_offset']
            }
        ],
        condition = IfCondition(LaunchConfiguration('perception'))
    )

    scan_ground_filter = Node(
        package="ground_segmentation",
        executable="scan_ground_filter_node",
        name="scan_ground_filter",
        remappings=[
            ("input", '/lidar_front/range_cropped_pointcloud'),
            ("output", '/lidar_front/no_ground_pointcloud')
        ],
        parameters=[
            {
                "global_slope_max_angle_deg": 8.0,
                "local_slope_max_angle_deg": 6.0,
                "split_points_distance_tolerance": 0.2,
                "split_height_distance": 0.2,
            },
            vehicle_info_param,
        ],
        condition = IfCondition(LaunchConfiguration('perception'))
    )

        
    # voxel_grid_based_euclidean_param_path = os.path.join(
    #     get_package_share_directory('ichthus_launch'), 'param/voxel_grid_based_euclidean_cluster.param.yaml' )
    # with open(voxel_grid_based_euclidean_param_path, "r") as f:
    #     voxel_grid_based_euclidean_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    # euclidean_cluster= Node(
    #     package='euclidean_cluster',
    #     executable="voxel_grid_based_euclidean_cluster_node",
    #     name="euclidean_cluster",
    #     remappings=[
    #         ("input", "/lidar_front/no_ground_pointcloud"),
    #         ("output", "/lidar_front/clusters_with_feature"),
    #     ],
    #     parameters=[voxel_grid_based_euclidean_param],
    # )

    dummy_perception_publisher = Node(
        package='dummy_perception_publisher',
        executable='dummy_perception_publisher_node',
        name='dummy_perception_publisher',
        remappings=[
            ('output/dynamic_object', '/lidar_front/labeled_clusters'),
            ('output/object_pose', 'debug/object_pose'),
            ('output/points_raw', 'debug/instance_pointcloud'),
            ('input/object', '/simulation/dummy_perception_publisher/object_info'),
            ('input/reset', 'input/reset'),
        ],

        parameters=[
            {
                'visible_range': 60.0,
                'detection_successful_rate' : 1.0,
                'enable_ray_tracing' : False,
                'use_object_recognition' : True,
                'object_centric_pointcloud' : False
            }
        ]
    )

    lidar_apollo_instance_segmentation_param_path = os.path.join(
        get_package_share_directory('ichthus_launch'), 'param/lidar_apollo_instance_segmentation.param.yaml' )

    with open(lidar_apollo_instance_segmentation_param_path, "r") as f:
        lidar_apollo_instance_segmentation_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    # apollo_lidar_model_param_path = PythonExpression(
    #     [
    #         "'param/vlp-16.param.yaml' if (",
    #         LaunchConfiguration("lidar_channel"),
    #         ") == 16 else 'param/vlp-64.param.yaml'",
    #     ]
    # )
    # print(apollo_lidar_model_param_path)
    
    # apollo_lidar_model_param_path = os.path.join(
    #     get_package_share_directory('ichthus_launch'), apollo_lidar_model_param_path'
    # )

    # with open(apollo_lidar_model_param_path, "r") as f:
    #     apollo_lidar_model_param = yaml.safe_load(f)["/**"]["ros__parameters"]
    # print(apollo_lidar_model_param)

    # lidar_apollo_instance_segmentation = Node(
    #     package='lidar_apollo_instance_segmentation',
    #     executable='lidar_apollo_instance_segmentation_node',
    #     name='lidar_apollo_instance_segmentation',
    #     remappings=[
    #         ('input/pointcloud', '/lidar_front/range_cropped_pointcloud'),
    #         ('output/labeled_clusters', '/lidar_front/labeled_clusters'),
    #     ],
    #     parameters=[
    #         {
    #         "z_offset" : -2,
    #         "target_frame" : "base_link",
    #         },
    #         lidar_apollo_instance_segmentation_param,
    #         apollo_lidar_model_param,
    #     ],

    #     condition = LaunchConfigurationEquals('lidar_detection_model', 'centerpoint') and LaunchConfigurationEquals("perception", "true")
    # )


    # lidar_centerpoint_param_path = os.path.join(
    #     get_package_share_directory('ichthus_launch'), 'param/lidar_centerpoint.param.yaml'
    # )

    # with open(lidar_centerpoint_param_path, "r") as f:
    #     lidar_centerpoint_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    # lidar_centerpoint = Node(
    #     package='lidar_centerpoint',
    #     executable='lidar_centerpoint_node',
    #     name='lidar_centerpoint',
    #     remappings=[
    #         ('~/input/pointcloud', '/lidar_front/range_cropped_pointcloud'),
    #         ('~/output/objects', '/centerpoint/objects')
    #     ],
    #     parameters=[
    #         lidar_centerpoint_param
    #     ],
    #     condition=IfCondition(
    #         PythonExpression([
    #             LaunchConfiguration('perception'),
    #             '==',
    #             'true',
    #             ' and ',
    #             LaunchConfiguration('lidar_detection_model'),
    #             '==',
    #             'centerpoint'
    #         ]))
    # )

    # shape_estimation = Node(
    #     package='shape_estimation',
    #     executable='shape_estimation',
    #     name='shape_estimation',
    #     remappings=[
    #         ('input', '/lidar_front/labeled_clusters'),
    #         ('objects','/lidar_front/objects_with_feature')
    #     ],
    #     parameters=[
    #         {
    #             'use_filter' : True,
    #             'use_corrector' : True,
    #             'use_vehicle_reference_yaw' : True
    #         }
    #     ]
    # )

    # detected_object_feature_remover = Node(
    #     package='detected_object_feature_remover',
    #     executable='detected_object_feature_remover',
    #     name='detected_object_feature_remover',
    #     remappings=[
    #         ('~/input', '/lidar_front/objects_with_feature'),
    #         ('~/output', '/lidar_front/objects')
    #     ]
    # )


    # detection_by_tracker = Node(
    #     package='detection_by_tracker',
    #     executable='detection_by_tracker',
    #     name='detection_by_tracker',
    #     remappings=[
    #         ('~/input/tracked_objects', '/perception/object_recognition/tracking/objects'),
    #         ('~/input/initial_objects', '/lidar_front/objects_with_feature'),
    #         ('~/output', '/detection_by_tracker/objects')
    #     ]
    # )

    # detected_object_validation = Node(
    #     package='detected_object_validation',
    #     executable='obstacle_pointcloud_based_validator_node',
    #     name='obstacle_pointcloud_base_validator',
    #     remappings=[
    #         ('~/input/detected_objects', '/lidar_front/objects'),
    #         ('~/input/obstacle_pointcloud', '/perception/obstacle_segmentation/pointcloud'),
    #         ('~/output/objects', '/lidar_front/validation/objects')
    #     ]
    # )

    # object_merger = Node(
    #     package='object_merger',
    #     executable='object_association_merger_node',
    #     name='object_merger',
    #     remappings=[
    #         ('input/object1', 'lidar_front/validation/objects'),
    #         ('input/object0', '/detection_by_tracker/objects'),
    #         ('output/object', '/objects')
    #     ]
    # )

    # occupancy_grid_map = Node(
    #     package="probabilistic_occupancy_grid_map",
    #     executable="pointcloud_based_occupancy_grid_map_node",
    #     name="occupancy_grid_map",
    #     remappings=[
    #         ("~/input/obstacle_pointcloud", '/lidar_front/no_ground_pointcloud'),
    #         ("~/input/raw_pointcloud", '/lidar_front/range_cropped_pointcloud'),
    #         ("~/output/occupancy_grid_map", '/perception/occupancy_grid_map/map'),
    #     ],
    #     parameters=[
    #         {
    #             "map_resolution": 0.5,
    #             "use_height_filter": False,
    #         }
    #     ],
    #     output='screen'
    # )

    # occupancy_grid_map_outlier_filter = Node(
    #     package="occupancy_grid_map_outlier_filter",
    #     executable="occupancy_grid_map_outlier_filter_node",
    #     name="occupancy_grid_map_outlier_filter",
    #     remappings=[
    #         ("~/input/occupancy_grid_map", "/perception/occupancy_grid_map/map"),
    #         ("~/input/pointcloud", '/lidar_front/no_ground_pointcloud'),
    #         ("~/output/pointcloud", '/perception/obstacle_segmentation/pointcloud'),
    #     ],
    # )

    # data_association_matrix_path = os.path.join(
    #     get_package_share_directory('ichthus_launch'), 'param/data_association_matrix.param.yaml' )

    # with open(data_association_matrix_path, "r") as f:
    #     data_association_matrix = yaml.safe_load(f)["/**"]["ros__parameters"]


    # tracker_setting_path = os.path.join(
    #     get_package_share_directory('ichthus_launch'), 'param/default_tracker.param.yaml' )

    # with open(tracker_setting_path, "r") as f:
    #     tracker_setting = yaml.safe_load(f)["/**"]["ros__parameters"]

    # multi_object_tracker = Node(
    #     package='multi_object_tracker',
    #     executable='multi_object_tracker',
    #     name='multi_object_tracker',
    #     remappings=[
    #         ('input', '/objects'),
    #         ('output', '/perception/object_recognition/tracking/objects'),
    #     ],
    #     parameters=[
    #         {
    #             'world_frame_id' : 'map',
    #             'publish_rate' : 10.0,
    #         },
    #         tracker_setting,
    #         data_association_matrix
    #     ],
    # )

    # map_based_prediction_param_path = os.path.join(
    #     get_package_share_directory('ichthus_launch'), 'param/map_based_prediction.param.yaml'
    # )
    
    # with open(map_based_prediction_param_path, "r") as f:
    #     map_based_prediction_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    # map_based_prediction = Node(
    #     package='map_based_prediction',
    #     executable='map_based_prediction',
    #     name='map_based_prediction',
    #     remappings=[
    #         ('/vector_map', '/map/vector_map'),
    #         ('objects', '/perception/object_recognition/objects')
    #     ],
    #     parameters=[
    #         map_based_prediction_param
    #     ]
    # )

    # mission_planner = Node(
    #     package="mission_planner",
    #     executable='mission_planner',
    #     name="mission_planner",
    #     remappings=[
    #         ("input/vector_map", "/map/vector_map"),
    #         ("input/goal_pose", "/planning/mission_planning/goal"),
    #         ("input/checkpoint", "/planning/mission_planning/checkpoint"),
    #         ("output/route", "/planning/mission_planning/route"),
    #         ("debug/route_marker", "/planning/mission_planning/route_marker"),
    #     ],
    #     parameters=[
    #         {
    #             "map_frame": "map",
    #             "base_link_frame": "base_link",
    #         }
    #     ],
    # )

    # goal_pose_visualizer = Node(
    #     package="mission_planner",
    #     executable="goal_pose_visualizer",
    #     name="goal_pose_visualizer",
    #     remappings=[
    #         ("input/route", "/planning/mission_planning/route"),
    #         ("output/goal_pose", "/planning/mission_planning/echo_back_goal_pose"),
    #     ],
    # )

    # scenario_selector = Node(
    #     package="scenario_selector",
    #     executable='scenario_selector',
    #     name='scenario_selector',
    #     remappings=[
    #         ('input/lane_driving/trajectory', '/planning/scenario_planning/lane_driving/trajectory'),
    #         ('input/parking/trajectory', '/planning/scenario_planning/parking/trajectory'),
    #         ('input/lanelet_map', '/map/vector_map'),
    #         ('input/route', '/planning/mission_planning/route'),
    #         ('input/odometry', '/lgsvl/gnss_odom'),
    #         ('is_parking_completed', '/planning/scenario_planning/parking/is_completed'),
    #         ('output/scenario' , '/planning/scenario_planning/scenario'),
    #         ('output/trajectory', '/planning/scenario_planning/scenario_selector/trajectory'),
    #     ],

    #     parameters=[
    #         {
    #             'update_rate' : 10.0,
    #             'th_max_message_delay_sec' : 1.0,
    #             'th_arrived_distance_m' : 1.0,
    #             'th_stopped_time_sec' : 1.0,
    #             'th_stopped_velocity_mps' : 0.01,
    #         }
    #     ]
    # )
    
    # param_path = os.path.join(get_package_share_directory('ichthus_launch'), 'param/common/motion_velocity_smoother/motion_velocity_smoother.param.yaml')

    # common_param_path = os.path.join(get_package_share_directory('ichthus_launch'), 'param/common/common.param.yaml')

    # param_path_param = DeclareLaunchArgument(
    #     'param_path',
    #     default_value=param_path
    # )

    # common_param_path_param = DeclareLaunchArgument(
    #     'common_param_path',
    #     default_value=common_param_path
    # )

    # external_velocity_limit_selector = Node(
    #     package='external_velocity_limit_selector',
    #     executable='external_velocity_limit_selector',
    #     name='external_velocity_limit_selector',
    #     remappings=[
    #         ("input/velocity_limit_from_api", "/planning/scenario_planning/max_velocity_default"),
    #         ("input/velocity_limit_from_internal" , "/planning/scenario_planning/max_velocity_candidates"),
    #         ("input/velocity_limit_clear_command_from_internal", "/planning/scenario_planning/clear_velocity_limit"),
    #         ("output/external_velocity_limit", '/planning/scenario_planning/max_velocity')
    #     ],
    #     parameters=[
    #         {
    #             'param_path' : LaunchConfiguration('param_path'),
    #             'common_param_path' : LaunchConfiguration('common_param_path')
    #         }
    #     ]
    # )

    # smoother_param_path = os.path.join(get_package_share_directory('ichthus_launch'), 'param/common/motion_velocity_smoother/JerkFiltered.param.yaml')
    
    # smoother_param_path_param = DeclareLaunchArgument(
    #     'smoother_param_path',
    #     default_value=smoother_param_path
    # )

    # motion_velocity_smoother = Node(
    #     package='motion_velocity_smoother',
    #     executable='motion_velocity_smoother',
    #     name='motion_velocity_smoother',
    #     remappings=[
    #         ("~/input/trajectory", '/planning/scenario_planning/scenario_selector/trajectory'),
    #         ("~/output/trajectory", '/planning/scenario_planning/trajectory'),
    #         ("~/input/external_velocity_limit_mps", "/planning/scenario_planning/max_velocity"),
    #         ("~/output/current_velocity_limit_mps", '/planning/scenario_planning/current_max_velocity'),
    #         ("/localization/kinematic_state", '/lgsvl/gnss_odom')
    #     ],
    #     parameters=[
    #         {
    #             "smoother_type" : 'JerkFiltered',
    #             'publish_debug_trajs' : True,
    #             'algorithm_type' : 'JerkFiltered',
    #             'param_path' : LaunchConfiguration('param_path'),
    #             'common_param_path' : LaunchConfiguration('common_param_path'),
    #             'smoother_param_path' : LaunchConfiguration('smoother_param_path'),
    #         }
    #     ]
    # )



    # side_shift_param_path = os.path.join(
    #     get_package_share_directory('ichthus_launch'), 'param/lane_driving/behavior_planning/behavior_path_planner/side_shift/side_shift.param.yaml'
    # )
    # with open(side_shift_param_path, "r") as f:
    #     side_shift_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    # avoidance_param_path = os.path.join(
    #     get_package_share_directory('ichthus_launch'), 'param/lane_driving/behavior_planning/behavior_path_planner/avoidance/avoidance.param.yaml'
    # )
    # with open(avoidance_param_path, "r") as f:
    #     avoidance_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    # lane_change_param_path = os.path.join(
    #     get_package_share_directory('ichthus_launch'), 'param/lane_driving/behavior_planning/behavior_path_planner/lane_change/lane_change.param.yaml'
    # )
    # with open(lane_change_param_path, "r") as f:
    #     lane_change_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    # lane_following_param_path = os.path.join(
    #     get_package_share_directory('ichthus_launch'), 'param/lane_driving/behavior_planning/behavior_path_planner/lane_following/lane_following.param.yaml'
    # )
    # with open(lane_following_param_path, "r") as f:
    #     lane_following_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    # pull_over_param_path = os.path.join(
    #     get_package_share_directory('ichthus_launch'), 'param/lane_driving/behavior_planning/behavior_path_planner/pull_over/pull_over.param.yaml'
    # )
    # with open(pull_over_param_path, "r") as f:
    #     pull_over_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    # pull_out_param_path = os.path.join(
    #     get_package_share_directory('ichthus_launch'), 'param/lane_driving/behavior_planning/behavior_path_planner/pull_out/pull_out.param.yaml'
    # )
    # with open(pull_out_param_path, "r") as f:
    #     pull_out_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    # behavior_path_planner_param_path = os.path.join(
    #     get_package_share_directory('ichthus_launch'), 'param/lane_driving/behavior_planning/behavior_path_planner/behavior_path_planner.param.yaml'
    # )
    # with open(behavior_path_planner_param_path, "r") as f:
    #     behavior_path_planner_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    # bt_tree_config_path = os.path.join(
    #     get_package_share_directory('ichthus_launch'), 'param/behavior_path_planner_tree.xml'
    # )

    # bt_tree_config_path_param = DeclareLaunchArgument(
    #     'bt_tree_config_path',
    #     default_value=bt_tree_config_path
    # )

    # behavior_path_planner = Node(
    #     package="behavior_path_planner",
    #     executable="behavior_path_planner",
    #     name="behavior_path_planner",
    #     remappings=[
    #         ("~/input/route", '/planning/mission_planning/route'),
    #         ("~/input/vector_map", "/map/vector_map"),
    #         ("~/input/perception", "/perception/object_recognition/objects"),
    #         ("~/input/odometry", "/lgsvl/gnss_odom"),
    #         ("~/input/scenario", "/planning/scenario_planning/scenario"),
    #         (
    #             "~/input/external_approval",
    #             "/planning/scenario_planning/lane_driving/behavior_planning/"
    #             "behavior_path_planner/path_change_approval",
    #         ),
    #         (
    #             "~/input/force_approval",
    #             "/planning/scenario_planning/lane_driving/behavior_planning/"
    #             "behavior_path_planner/path_change_force",
    #         ),
    #         ("~/output/path", "path_with_lane_id"),
    #         (
    #             "~/output/ready",
    #             "/planning/scenario_planning/lane_driving/behavior_planning/"
    #             "behavior_path_planner/ready_module",
    #         ),
    #         (
    #             "~/output/running",
    #             "/planning/scenario_planning/lane_driving/behavior_planning/"
    #             "behavior_path_planner/running_modules",
    #         ),
    #         (
    #             "~/output/force_available",
    #             "/planning/scenario_planning/lane_driving/behavior_planning/"
    #             "behavior_path_planner/force_available",
    #         ),
    #         ("~/output/turn_indicators_cmd", "/planning/turn_indicators_cmd"),
    #         ("~/output/hazard_lights_cmd", "/planning/hazard_lights_cmd"),
    #     ],
    #     parameters=[
    #         side_shift_param,
    #         avoidance_param,
    #         lane_change_param,
    #         lane_following_param,
    #         pull_over_param,
    #         pull_out_param,
    #         behavior_path_planner_param,
    #         vehicle_info_param,

    #         {
    #             "bt_tree_config_path":  LaunchConfiguration("bt_tree_config_path"),
    #             "planning_hz" : 10.0
    #         }
    #     ],
    # )

    # # obstacle avoidance planner
    # obstacle_avoidance_planner_param_path = os.path.join(
    #     get_package_share_directory('ichthus_launch'), 'param/lane_driving/motion_planning/obstacle_avoidance_planner/obstacle_avoidance_planner.param.yaml'
    # )
    # with open(obstacle_avoidance_planner_param_path, "r") as f:
    #     obstacle_avoidance_planner_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    # obstacle_avoidance_planner= Node(
    #     package="obstacle_avoidance_planner",
    #     executable="obstacle_avoidance_planner_node",
    #     name="obstacle_avoidance_planner",
    #     remappings=[
    #         ("~/input/objects", "/perception/object_recognition/objects"),
    #         ('localization/kinematic_state', '/lgsvl/gnss_odom'),
    #         ("~/input/path", '/planning/scenario_planning/lane_driving/behavior_planning/path'),
    #         ("~/output/path", "obstacle_avoidance_planner/trajectory"),
    #     ],
    #     parameters=[
    #         obstacle_avoidance_planner_param,
    #         vehicle_info_param,
    #         {"is_showing_debug_info": False},
    #         {"is_stopping_if_outside_drivable_area": True},
    #     ],
    # )

    # surround_obstacle_checker_param_path = os.path.join(
    #    get_package_share_directory('ichthus_launch'), 'param/lane_driving/motion_planning/surround_obstacle_checker/surround_obstacle_checker.param.yaml'
    # )
    # with open(surround_obstacle_checker_param_path, "r") as f:
    #     surround_obstacle_checker_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    # surround_obstacle_checker = Node(
    #     package="surround_obstacle_checker",
    #     executable="surround_obstacle_checker_node",
    #     name="surround_obstacle_checker",
    #     remappings=[
    #         ("~/output/no_start_reason", "/planning/scenario_planning/status/no_start_reason"),
    #         ("~/output/stop_reasons", "/planning/scenario_planning/status/stop_reasons"),
    #         ("~/output/max_velocity", "/planning/scenario_planning/max_velocity_candidates"),
    #         (
    #             "~/output/velocity_limit_clear_command",
    #             "/planning/scenario_planning/clear_velocity_limit",
    #         ),
    #         (
    #             "~/input/pointcloud",
    #             "/perception/obstacle_segmentation/pointcloud",
    #         ),
    #         ("~/input/objects", "/perception/object_recognition/objects"),
    #         ("~/input/odometry", "/lgsvl/gnss_odom"),
    #     ],
    #     parameters=[
    #         surround_obstacle_checker_param,
    #         vehicle_info_param,
    #     ],
    # )

    # obstacle_stop_planner_param_path = os.path.join(
    #     get_package_share_directory('ichthus_launch'), 'param/lane_driving/motion_planning/obstacle_stop_planner/obstacle_stop_planner.param.yaml'
    # )
    # with open(obstacle_stop_planner_param_path, "r") as f:
    #     obstacle_stop_planner_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    # obstacle_stop_planner_acc_param_path = os.path.join(
    #     get_package_share_directory('ichthus_launch'), 'param/lane_driving/motion_planning/obstacle_stop_planner/adaptive_cruise_control.param.yaml'
    # )
    # with open(obstacle_stop_planner_acc_param_path, "r") as f:
    #     obstacle_stop_planner_acc_param = yaml.safe_load(f)["/**"]["ros__parameters"]
    
    # common_param_path = os.path.join(
    #     get_package_share_directory("ichthus_launch"), 'param/common/common.param.yaml'
    # )
    # with open(common_param_path, "r") as f:
    #     common_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    # obstacle_stop_planner = Node(
    #     package="obstacle_stop_planner",
    #     executable="obstacle_stop_planner_node",
    #     name="obstacle_stop_planner",
    #     namespace="",
    #     remappings=[
    #         ("~/output/stop_reason", "/planning/scenario_planning/status/stop_reason"),
    #         ("~/output/stop_reasons", "/planning/scenario_planning/status/stop_reasons"),
    #         ("~/output/max_velocity", "/planning/scenario_planning/max_velocity_candidates"),
    #         (
    #             "~/output/velocity_limit_clear_command",
    #             "/planning/scenario_planning/clear_velocity_limit",
    #         ),
    #         ("~/output/trajectory", "/planning/scenario_planning/lane_driving/trajectory"),
    #         (
    #             "~/input/pointcloud",
    #             "/perception/obstacle_segmentation/pointcloud",
    #         ),
    #         ("~/input/objects", "/perception/object_recognition/objects"),
    #         ("~/input/odometry", "/lgsvl/gnss_odom"),
    #         ("~/input/trajectory", "obstacle_avoidance_planner/trajectory"),
    #     ],
    #     parameters=[
    #         common_param,
    #         obstacle_stop_planner_param,
    #         obstacle_stop_planner_acc_param,
    #         vehicle_info_param,
    #         {"enable_slow_down": False},
    #     ],
    #     # extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    # )

    # freespace_planner_param_path = os.path.join(
    #     get_package_share_directory("ichthus_launch"), 'param/parking/freespace_planner/freespace_planner.param.yaml'
    # )
    # with open(freespace_planner_param_path, "r") as f:
    #     freespace_planner_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    # costmap_generator = Node(
    #     package="costmap_generator",
    #     executable="costmap_generator",
    #     name="costmap_generator",
    #     remappings=[
    #         ("~/input/objects", "/perception/object_recognition/objects"),
    #         (
    #             "~/input/points_no_ground",
    #             "/perception/obstacle_segmentation/pointcloud",
    #         ),
    #         ("~/input/vector_map", "/map/vector_map"),
    #         ("~/input/scenario", "/planning/scenario_planning/scenario"),
    #         ("~/output/grid_map", "costmap_generator/grid_map"),
    #         ("~/output/occupancy_grid", "costmap_generator/occupancy_grid"),
    #     ],
    #     parameters=[
    #         {
    #             "costmap_frame": "map",
    #             "vehicle_frame": "base_link",
    #             "map_frame": "map",
    #             "update_rate": 10.0,
    #             "use_wayarea": True,
    #             "use_objects": True,
    #             "use_points": True,
    #             "grid_min_value": 0.0,
    #             "grid_max_value": 1.0,
    #             "grid_resolution": 0.2,
    #             "grid_length_x": 70.0,
    #             "grid_length_y": 70.0,
    #             "grid_position_x": 0.0,
    #             "grid_position_y": 0.0,
    #             "maximum_lidar_height_thres": 0.3,
    #             "minimum_lidar_height_thres": -2.2,
    #             "expand_polygon_size": 1.0,
    #             "size_of_expansion_kernel": 9,
    #         },
    #         vehicle_info_param,
    #     ],
    # )

    # freespace_planner = Node(
    #     package="freespace_planner",
    #     executable="freespace_planner",
    #     name="freespace_planner",
    #     remappings=[
    #         ("~/input/route", "/planning/mission_planning/route"),
    #         ("~/input/occupancy_grid", "costmap_generator/occupancy_grid"),
    #         ("~/input/scenario", "/planning/scenario_planning/scenario"),
    #         ("~/input/odometry", "/lgsvl/gnss_odom"),
    #         ("~/output/trajectory", "/planning/scenario_planning/parking/trajectory"),
    #         ("is_completed", "/planning/scenario_planning/parking/is_completed"),
    #     ],
    #     parameters=[
    #         freespace_planner_param,
    #         vehicle_info_param,
    #     ],
    # )

    # blind_spot_param_path = os.path.join(
    #     get_package_share_directory('ichthus_launch'), 'param/lane_driving/behavior_planning/behavior_velocity_planner/', 
    #     'blind_spot.param.yaml'
    # )
    # with open(blind_spot_param_path, "r") as f:
    #     blind_spot_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    # crosswalk_param_path = os.path.join(
    #     get_package_share_directory('ichthus_launch'), 'param/lane_driving/behavior_planning/behavior_velocity_planner/',
    #     "crosswalk.param.yaml",
    # )
    # with open(crosswalk_param_path, "r") as f:
    #     crosswalk_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    # detection_area_param_path = os.path.join(
    #     get_package_share_directory('ichthus_launch'), 'param/lane_driving/behavior_planning/behavior_velocity_planner/',
    #     "detection_area.param.yaml",
    # )
    # with open(detection_area_param_path, "r") as f:
    #     detection_area_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    # intersection_param_path = os.path.join(
    #     get_package_share_directory('ichthus_launch'), 'param/lane_driving/behavior_planning/behavior_velocity_planner/',
    #     "intersection.param.yaml",
    # )
    # with open(intersection_param_path, "r") as f:
    #     intersection_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    # stop_line_param_path = os.path.join(
    #     get_package_share_directory('ichthus_launch'), 'param/lane_driving/behavior_planning/behavior_velocity_planner/',
    #     "stop_line.param.yaml",
    # )
    # with open(stop_line_param_path, "r") as f:
    #     stop_line_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    # traffic_light_param_path = os.path.join(
    #     get_package_share_directory('ichthus_launch'), 'param/lane_driving/behavior_planning/behavior_velocity_planner/',
    #     "traffic_light.param.yaml",
    # )
    # with open(traffic_light_param_path, "r") as f:
    #     traffic_light_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    # virtual_traffic_light_param_path = os.path.join(
    #     get_package_share_directory('ichthus_launch'), 'param/lane_driving/behavior_planning/behavior_velocity_planner/',
    #     "virtual_traffic_light.param.yaml",
    # )
    # with open(virtual_traffic_light_param_path, "r") as f:
    #     virtual_traffic_light_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    # occlusion_spot_param_path = os.path.join(
    #     get_package_share_directory('ichthus_launch'), 'param/lane_driving/behavior_planning/behavior_velocity_planner/',
    #     "occlusion_spot.param.yaml",
    # )
    # with open(occlusion_spot_param_path, "r") as f:
    #     occlusion_spot_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    # no_stopping_area_param_path = os.path.join(
    #     get_package_share_directory('ichthus_launch'), 'param/lane_driving/behavior_planning/behavior_velocity_planner/',
    #     "no_stopping_area.param.yaml",
    # )
    # with open(no_stopping_area_param_path, "r") as f:
    #     no_stopping_area_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    # behavior_velocity_planner = Node(
    #     package="behavior_velocity_planner",
    #     executable="behavior_velocity_planner_node",
    #     name="behavior_velocity_planner",
    #     remappings=[
    #         ("~/input/path_with_lane_id", "path_with_lane_id"),
    #         ("~/input/vector_map", "/map/vector_map"),
    #         ("~/input/vehicle_odometry", "/lgsvl/gnss_odom"),
    #         ("~/input/dynamic_objects", "/perception/object_recognition/objects"),
    #         (
    #             "~/input/no_ground_pointcloud",
    #             "/perception/obstacle_segmentation/pointcloud",
    #         ),
    #         (
    #             "~/input/traffic_signals",
    #             "/perception/traffic_light_recognition/traffic_signals",
    #         ),
    #         (
    #             "~/input/external_traffic_signals",
    #             "/external/traffic_light_recognition/traffic_signals",
    #         ),
    #         ("~/input/virtual_traffic_light_states", "/awapi/tmp/virtual_traffic_light_states"),
    #         (
    #             "~/input/occupancy_grid",
    #             "/perception/occupancy_grid_map/map",
    #         ),
    #         ("~/output/path", "/planning/scenario_planning/lane_driving/behavior_planning/path"),
    #         ("~/output/stop_reasons", "/planning/scenario_planning/status/stop_reasons"),
    #         (
    #             "~/output/infrastructure_commands",
    #             "/planning/scenario_planning/status/infrastructure_commands",
    #         ),
    #         ("~/output/traffic_signal", "debug/traffic_signal"),
    #     ],
    #     parameters=[
    #         {
    #             "launch_stop_line": True,
    #             "launch_crosswalk": True,
    #             "launch_traffic_light": True,
    #             "launch_intersection": True,
    #             "launch_blind_spot": True,
    #             "launch_detection_area": True,
    #             "launch_virtual_traffic_light": True,
    #             "launch_occlusion_spot": True,
    #             "launch_no_stopping_area": True,
    #             "forward_path_length": 1000.0,
    #             "backward_path_length": 5.0,
    #             "max_accel": -2.8,
    #             "delay_response_time": 1.3,
    #         },
    #         blind_spot_param,
    #         crosswalk_param,
    #         detection_area_param,
    #         intersection_param,
    #         stop_line_param,
    #         traffic_light_param,
    #         virtual_traffic_light_param,
    #         occlusion_spot_param,
    #         no_stopping_area_param,
    #         vehicle_info_param,
    #     ],
    # )


    # lat_controller_param_path = os.path.join(
    #     get_package_share_directory('ichthus_launch'), 'param/lateral_controller.param.yaml'
    # )

    # lon_controller_param_path = os.path.join(
    #     get_package_share_directory('ichthus_launch'), 'param/longitudinal_controller.param.yaml'
    # )

    # latlon_muxer_param_path = os.path.join(
    #     get_package_share_directory('ichthus_launch'), 'param/latlon_muxer.param.yaml'
    # )

    # lane_departure_checker_param_path = os.path.join(
    #     get_package_share_directory('ichthus_launch'), 'param/lane_departure_checker.param.yaml'
    # )

    # vehicle_cmd_gate_param_path = os.path.join(
    #     get_package_share_directory('ichthus_launch'), 'param/vehicle_cmd_gate.param.yaml'
    # )

    # with open(lat_controller_param_path, "r") as f:
    #     lat_controller_param = yaml.safe_load(f)["/**"]["ros__parameters"]
    # with open(lon_controller_param_path, "r") as f:
    #     lon_controller_param = yaml.safe_load(f)["/**"]["ros__parameters"]
    # with open(latlon_muxer_param_path, "r") as f:
    #     latlon_muxer_param = yaml.safe_load(f)["/**"]["ros__parameters"]
    # with open(lane_departure_checker_param_path, "r") as f:
    #     lane_departure_checker_param = yaml.safe_load(f)["/**"]["ros__parameters"]
    # with open(vehicle_cmd_gate_param_path, "r") as f:
    #     vehicle_cmd_gate_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    # lat_controller = Node(
    #     package="trajectory_follower_nodes",
    #     executable="lateral_controller_node_exe",
    #     name="lateral_controller",
    #     remappings=[
    #         ("~/input/reference_trajectory", "/planning/scenario_planning/trajectory"),
    #         ("~/input/current_odometry", "/lgsvl/gnss_odom"),
    #         ("~/input/current_steering", "/vehicle/status/steering_status"),
    #         ("~/output/control_cmd", "lateral/control_cmd"),
    #         ("~/output/predicted_trajectory", "lateral/predicted_trajectory"),
    #         ("~/output/diagnostic", "lateral/diagnostic"),
    #     ],
    #     parameters=[
    #         lat_controller_param,
    #         vehicle_info_param,
    #     ],
    # )

    # lon_controller = Node(
    #     package="trajectory_follower_nodes",
    #     executable="longitudinal_controller_node_exe",
    #     name="longitudinal_controller",
    #     remappings=[
    #         ("~/input/current_trajectory", "/planning/scenario_planning/trajectory"),
    #         ("~/input/current_odometry", "/lgsvl/gnss_odom"),
    #         ("~/output/control_cmd", "longitudinal/control_cmd"),
    #         ("~/output/slope_angle", "longitudinal/slope_angle"),
    #         ("~/output/diagnostic", "longitudinal/diagnostic"),
    #     ],
    #     parameters=[
    #         lon_controller_param,
    #         vehicle_info_param,
    #         {
    #             "show_debug_info": False,
    #             "enable_pub_debug": True
    #         },
    #     ],
    # )

    # latlon_muxer= Node(
    #     package="trajectory_follower_nodes",
    #     executable="latlon_muxer_node_exe",
    #     name="latlon_muxer",
    #     remappings=[
    #         ("~/input/lateral/control_cmd", "lateral/control_cmd"),
    #         ("~/input/longitudinal/control_cmd", "longitudinal/control_cmd"),
    #         ("~/output/control_cmd", "control/trajectory_follower/control_cmd"),
    #     ],
    #     parameters=[
    #         latlon_muxer_param,
    #     ],
    # )

    # lane_departure_checker= Node(
    #     package="lane_departure_checker",
    #     executable="lane_departure_checker_node",
    #     name="lane_departure_checker",
    #     remappings=[
    #         ("~/input/odometry", "/lgsvl/gnss_odom"),
    #         ("~/input/lanelet_map_bin", "/map/vector_map"),
    #         ("~/input/route", "/planning/mission_planning/route"),
    #         ("~/input/reference_trajectory", "/planning/scenario_planning/trajectory"),
    #         (
    #             "~/input/predicted_trajectory",
    #             "/lateral/predicted_trajectory",
    #         ),
    #     ],
    #     parameters=[lane_departure_checker_param, vehicle_info_param],
    # )

    # shift_decider = Node(
    #     package="shift_decider",
    #     executable="shift_decider",
    #     name="shift_decider",
    #     remappings=[
    #         ("input/control_cmd", "/control/trajectory_follower/control_cmd"),
    #         ("output/gear_cmd", "/control/shift_decider/gear_cmd"),
    #     ],
    # )

    # vehicle_cmd_gate = Node(
    #     package="vehicle_cmd_gate",
    #     executable="vehicle_cmd_gate",
    #     name="vehicle_cmd_gate",
    #     remappings=[
    #         ("input/emergency_state", "/system/emergency/emergency_state"),
    #         ("input/steering", "/vehicle/status/steering_status"),
    #         ("input/auto/control_cmd", "/control/trajectory_follower/control_cmd"),
    #         ("input/auto/turn_indicators_cmd", "/planning/turn_indicators_cmd"),
    #         ("input/auto/hazard_lights_cmd", "/planning/hazard_lights_cmd"),
    #         ("input/auto/gear_cmd", "/control/shift_decider/gear_cmd"),
    #         ("input/external/control_cmd", "/external/selected/control_cmd"),
    #         ("input/external/turn_indicators_cmd", "/external/selected/turn_indicators_cmd"),
    #         ("input/external/hazard_lights_cmd", "/external/selected/hazard_lights_cmd"),
    #         ("input/external/gear_cmd", "/external/selected/gear_cmd"),
    #         ("input/external_emergency_stop_heartbeat", "/external/selected/heartbeat"),
    #         ("input/gate_mode", "/control/gate_mode_cmd"),
    #         ("input/emergency/control_cmd", "/system/emergency/control_cmd"),
    #         ("input/emergency/hazard_lights_cmd", "/system/emergency/hazard_lights_cmd"),
    #         ("input/emergency/gear_cmd", "/system/emergency/gear_cmd"),
    #         ("output/vehicle_cmd_emergency", "/control/command/emergency_cmd"),
    #         ("output/control_cmd", "/control/command/control_cmd"),
    #         ("output/gear_cmd", "/control/command/gear_cmd"),
    #         ("output/turn_indicators_cmd", "/control/command/turn_indicators_cmd"),
    #         ("output/hazard_lights_cmd", "/control/command/hazard_lights_cmd"),
    #         ("output/gate_mode", "/control/current_gate_mode"),
    #         ("output/engage", "/api/autoware/get/engage"),
    #         ("output/external_emergency", "/api/autoware/get/emergency"),
    #         ("~/service/engage", "/api/autoware/set/engage"),
    #         ("~/service/external_emergency", "/api/autoware/set/emergency"),
    #         # TODO(Takagi, Isamu): deprecated
    #         ("input/engage", "/autoware/engage"),
    #         ("~/service/external_emergency_stop", "~/external_emergency_stop"),
    #         ("~/service/clear_external_emergency_stop", "~/clear_external_emergency_stop"),
    #     ],
    #     parameters=[
    #         vehicle_cmd_gate_param,
    #         vehicle_info_param,
    #         {
    #             "use_emergency_handling": False,
    #             "use_external_emergency_stop": True,
    #             "use_start_request": False
    #         },
    #     ],
    # )

    # state_report = Node(
    #     package='state_report',
    #     executable='state_report_node',
    #     name='state_report',
    #     remappings=[
    #         ('input/vehicle_kinematic_state', '/vehicle/vehicle_kinematic_state'),
    #         ('input/turn_indicators_command', '/control/command/turn_indicators_cmd'),
    #         ('output/vehicle_velocity_report', '/vehicle/status/velocity_status'),
    #         ('output/turn_indicators_report', '/vehicle/status/turn_indicators_status'),
    #         ('output/steering_report', '/vehicle/status/steering_status')
    #     ]
    # )


    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', ichthus_launch_pkg_prefix + '/rviz/ichthus2.rviz'],
    )

    # path_change_approval_pub = ExecuteProcess(
    #     cmd=[
    #         "ros2","topic","pub", "/planning/scenario_planning/lane_driving/behavior_planning/"
    #         "behavior_path_planner/path_change_approval", "tier4_planning_msgs/msg/Approval", 
    #         "{approval: true}", "-r", "10",
    #     ]
    # ),

    # engage_pub = ExecuteProcess(
    #     cmd=[
    #         "ros2", "topic", "pub", "/autoware/engage", "autoware_auto_vehicle_msgs/msg/Engage",
    #         "{engage: true}","-r", "10", "-t", "100",
    #     ]
    # ),



    return LaunchDescription([
        *launch_arguments,
        lgsvl_interface_node_param,
        lgsvl_interface,
        urdf_publisher,
        image_transport_decompressor,
        tensorrt_yolo,
        lanelet2_map_path_param,
        lanelet2_map_loader,
        lanelet2_map_visualization,
        map_odom_publisher,
        # crop_box_filter,
        # scan_ground_filter,
        # occupancy_grid_map,
        # occupancy_grid_map_outlier_filter,
        # lidar_apollo_instance_segmentation,
        # shape_estimation,
        # detected_object_feature_remover,
        # lidar_centerpoint,
        # detected_object_validation,
        # detection_by_tracker,
        # object_merger,
        # multi_object_tracker,
        # map_based_prediction,

        ###planner test###

        # mission_planner,
        # goal_pose_visualizer,
        # param_path_param,
        # common_param_path_param,
        # scenario_selector,
        # external_velocity_limit_selector,
        # smoother_param_path_param,
        # motion_velocity_smoother,
        # bt_tree_config_path_param,
        # behavior_path_planner,
        # behavior_velocity_planner,
        # obstacle_avoidance_planner,
        # # surround_obstacle_checker,
        # obstacle_stop_planner,
        # lon_controller,
        # lat_controller,
        # latlon_muxer,
        # #shift_decider,
        # vehicle_cmd_gate,
        # state_report,
        rviz2,

        # pointcloud_map_path_param,
        # pointcloud_map_loader,
        # map_hash_generator,
        # map_tf_generator,
        # viewer_map_publisher,
        # approximate_downsample_filter,
        # outlier_filter,
        # euclidean_cluster,
        # dummy_perception_publisher,
    ])


# # lane_departure_checker
# # # # costmap_generator,
        # # # # freespace_planner,