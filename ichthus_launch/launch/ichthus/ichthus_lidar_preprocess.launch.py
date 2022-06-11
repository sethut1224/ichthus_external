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
        self.use_multi_lidar = LaunchConfiguration('use_multi_lidar').perform(self.context)
        self.raw_pointcloud = LaunchConfiguration('raw_pointcloud_topic').perform(self.context)

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

    def lidar_preprocess(self, raw_pointcloud, range_cropped_pointcloud, no_ground_pointcloud, grid_map, obstacle_segmentation_pointcloud):
        
        components = []

        concatenate_filter = ComposableNode(
            package='pointcloud_preprocessor',
            plugin="pointcloud_preprocessor::PointCloudConcatenateDataSynchronizerComponent",
            name="concatenate_filter",
            remappings=[
                ("output", "/merged_cloud")
            ],
            parameters=[
                {
                    "input_topics": ['/lidar_front/points_raw', '/lidar_rear/points_raw'],
                    "output_frame": "base_link",
                    "approximate_sync": True,
                    'use_sim_time' : LaunchConfiguration('use_sim_time')
                }
            ],
            extra_arguments=[{"use_intra_process_comms": True}],
        )

        crop_box_filter= ComposableNode(
            package='pointcloud_preprocessor',
            plugin="pointcloud_preprocessor::CropBoxFilterComponent",
            name="crop_box_filter",
            remappings=[
                ("input",  raw_pointcloud),
                ('output', range_cropped_pointcloud)
            ],
            parameters=[
                {
                    "input_frame": 'base_link',
                    "output_frame": 'base_link',
                    'min_x' : -60.0,
                    'max_x' : 100.0,
                    'min_y' : -60.0,
                    'max_y' : 60.0,
                    "min_z": -0.9,
                    "max_z": self.vehicle_info['max_height_offset'],
                    'max_queue_size' : 1,
                    'use_sim_time' : LaunchConfiguration('use_sim_time')
                }
            ],
            extra_arguments=[{"use_intra_process_comms": True}],
        )

        scan_ground_filter = ComposableNode(
            package="ground_segmentation",
            plugin="ground_segmentation::ScanGroundFilterComponent",
            name="scan_ground_filter",
            remappings=[
                ("input", range_cropped_pointcloud),
                ("output", no_ground_pointcloud)
            ],
            parameters=[
                {
                    "global_slope_max_angle_deg": 10.0,
                    "local_slope_max_angle_deg": 30.0,
                    "split_points_distance_tolerance": 0.2,
                    "split_height_distance": 0.3,
                    'max_queue_size' : 1,
                    'use_virtual_ground_point' : True,
                    'use_sim_time' : LaunchConfiguration('use_sim_time')
                },
                self.vehicle_info
            ],
            extra_arguments=[{"use_intra_process_comms": True}],
        )

        occupancy_grid_map = ComposableNode(
            package="probabilistic_occupancy_grid_map",
            plugin="occupancy_grid_map::PointcloudBasedOccupancyGridMapNode",
            name="occupancy_grid_map",
            remappings=[
                ("~/input/obstacle_pointcloud", no_ground_pointcloud),
                ("~/input/raw_pointcloud", range_cropped_pointcloud),
                ("~/output/occupancy_grid_map", grid_map),
            ],
            parameters=[
                {
                    "map_resolution": 0.4,
                    "use_height_filter": False,
                    'use_sim_time' : LaunchConfiguration('use_sim_time')
                }
            ],
            extra_arguments=[{"use_intra_process_comms": True}],
        )

        occupancy_grid_map_outlier_filter = ComposableNode(
            package="occupancy_grid_map_outlier_filter",
            plugin="occupancy_grid_map_outlier_filter::OccupancyGridMapOutlierFilterComponent",
            name="occupancy_grid_map_outlier_filter",
            remappings=[
                ("~/input/occupancy_grid_map", grid_map),
                ("~/input/pointcloud", no_ground_pointcloud),
                ("~/output/pointcloud", obstacle_segmentation_pointcloud),
            ],
            parameters=[
                {
                    'use_sim_time' : LaunchConfiguration('use_sim_time')
                }
            ],
            extra_arguments=[{"use_intra_process_comms": True}],
        )

        if self.use_multi_lidar == 'true' or self.use_multi_lidar == 'True':
            return [concatenate_filter, crop_box_filter, scan_ground_filter, occupancy_grid_map, occupancy_grid_map_outlier_filter]
        else:
            return [crop_box_filter, scan_ground_filter, occupancy_grid_map, occupancy_grid_map_outlier_filter]


def launch_setup(context, *args, **kwargs):
    pipeline = Sensing(context)

    nodes = []

    raw_pointcloud = LaunchConfiguration('raw_pointcloud_topic').perform(context)
    range_cropped_pointcloud = LaunchConfiguration('range_cropped_pointcloud_topic').perform(context)
    no_ground_pointcloud = LaunchConfiguration('no_ground_pointcloud_topic').perform(context)
    grid_map = LaunchConfiguration('grid_map_topic').perform(context)
    obstacle_segmentation_pointcloud = LaunchConfiguration('obstacle_segmentation_pointcloud_topic').perform(context)
    lidar_preprocess_nodes = pipeline.lidar_preprocess(raw_pointcloud, range_cropped_pointcloud, no_ground_pointcloud, grid_map, obstacle_segmentation_pointcloud)

    nodes.extend(lidar_preprocess_nodes)

    container = ComposableNodeContainer(
        name="pointcloud_preprocessor_container",
        namespace="",
        package="rclcpp_components",
        executable=LaunchConfiguration('container_executable'),
        composable_node_descriptions=nodes,
        output="screen",
    )

    return [container]

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

    add_launch_arg('raw_pointcloud_topic', '/merged_cloud'),
    add_launch_arg('no_ground_pointcloud_topic', '/no_ground_pointcloud'),
    add_launch_arg('range_cropped_pointcloud_topic', '/range_cropped_pointcloud')
    add_launch_arg('grid_map_topic', '/perception/occupancy_grid_map/map')
    add_launch_arg('obstacle_segmentation_pointcloud_topic', '/perception/obstacle_segmentation/pointcloud')
    add_launch_arg('vehicle_info_param_path', vehicle_info_param_path_default),
    add_launch_arg('use_sim_time', 'False')
    add_launch_arg('use_multi_lidar', 'False')
    add_launch_arg('use_multithread', 'True')

    set_container_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container",
        condition=UnlessCondition(LaunchConfiguration("use_multithread")),
    )

    set_container_mt_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container_mt",
        condition=IfCondition(LaunchConfiguration("use_multithread")),
    )

    return launch.LaunchDescription(
        launch_arguments
        +[set_container_executable, set_container_mt_executable]
        + [OpaqueFunction(function=launch_setup)]
    )