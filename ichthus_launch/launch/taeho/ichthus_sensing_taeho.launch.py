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
        self.raw_pointcloud = '/lidar_front/points_raw' if self.use_multi_lidar == 'false' else 'concatenated/pointcloud'

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

    def lidar_preprocess(self, raw_pointcloud, range_cropped_pointcloud, no_ground_pointcloud):
        
        concatenate_filter = Node(
            package='pointcloud_preprocessor',
            executable="concatenate_data_node",
            name="concatenate_filter",
            remappings=[
                ("output", "concatenated/pointcloud")
            ],
            parameters=[
                {
                    "input_topics": ['/lidar_front/points_raw', '/lidar_rear/points_raw'],
                    "output_frame": "base_link",
                    "approximate_sync": True,
                    'use_sim_time' : LaunchConfiguration('use_sim_time')
                }
            ],
            condition=IfCondition(LaunchConfiguration('use_multi_lidar'))
        )

        crop_box_filter= Node(
            package='pointcloud_preprocessor',
            executable="crop_box_filter_node",
            name="crop_box_filter",
            remappings=[
                ("input",  self.raw_pointcloud),
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
                    "min_z": -0.5,
                    "max_z": self.vehicle_info['max_height_offset'],
                    'max_queue_size' : 1,
                    'use_sim_time' : LaunchConfiguration('use_sim_time')
                }
            ],
        )

        voxel_grid_downsample_filter_param_path = LaunchConfiguration('voxel_grid_downsample_filter_param_path').perform(self.context)

        with open(voxel_grid_downsample_filter_param_path, "r") as f:
            voxel_grid_downsample_filter_param = yaml.safe_load(f)["/**"]["ros__parameters"]
        
        voxel_grid_downsample_filter = Node(
            package="pointcloud_preprocessor",
            executable='voxel_grid_downsample_filter_node',
            name="voxel_grid_downsample_filter",
            remappings=[
                ("input", self.raw_pointcloud),
                ("output", "voxel_grid_downsample/pointcloud"),
            ],

            parameters=[
                voxel_grid_downsample_filter_param,
                {
                    'voxel_size_x' : 2.0,
                    'voxel_size_y' : 2.0,
                    'voxel_size_z' : 2.0,
                    'use_sim_time' : LaunchConfiguration('use_sim_time'),
                }
            ],
        )

        scan_ground_filter = Node(
            package="ground_segmentation",
            executable="scan_ground_filter_node",
            name="scan_ground_filter",
            remappings=[
                ("input", range_cropped_pointcloud),
                ("output", no_ground_pointcloud)
            ],
            parameters=[
                {
                    "global_slope_max_angle_deg": 8.0,
                    "local_slope_max_angle_deg": 6.0,
                    "split_points_distance_tolerance": 0.2,
                    "split_height_distance": 0.2,
                    'max_queue_size' : 1,
                    'use_sim_time' : LaunchConfiguration('use_sim_time')
                },
                self.vehicle_info
            ],
        )

        return [concatenate_filter, crop_box_filter, voxel_grid_downsample_filter, scan_ground_filter]
        

    def occupancy_grid_map(self, no_ground_pointcloud, range_cropped_pointcloud, grid_map, obstacle_segmentation_pointcloud):

        occupancy_grid_map = Node(
            package="probabilistic_occupancy_grid_map",
            executable="pointcloud_based_occupancy_grid_map_node",
            name="occupancy_grid_map",
            remappings=[
                ("~/input/obstacle_pointcloud", no_ground_pointcloud),
                ("~/input/raw_pointcloud", range_cropped_pointcloud),
                ("~/output/occupancy_grid_map", grid_map),
            ],
            parameters=[
                {
                    "map_resolution": 0.5,
                    "use_height_filter": False,
                    'use_sim_time' : LaunchConfiguration('use_sim_time')
                }
            ],
            output='screen'
        )

        occupancy_grid_map_outlier_filter = Node(
            package="occupancy_grid_map_outlier_filter",
            executable="occupancy_grid_map_outlier_filter_node",
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
            ]
        )

        return [occupancy_grid_map, occupancy_grid_map_outlier_filter]

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
            ]
        )

        return [imu_corrector]

def launch_setup(context, *args, **kwargs):
    pipeline = Sensing(context)

    nodes = list()

    raw_pointcloud = LaunchConfiguration('raw_pointcloud_topic').perform(context)
    range_cropped_pointcloud = LaunchConfiguration('range_cropped_pointcloud_topic').perform(context)
    no_ground_pointcloud = LaunchConfiguration('no_ground_pointcloud_topic').perform(context)
    grid_map = LaunchConfiguration('grid_map_topic').perform(context)
    obstacle_segmentation_pointcloud = LaunchConfiguration('obstacle_segmentation_pointcloud_topic').perform(context)

    lidar_preprocess_nodes = pipeline.lidar_preprocess(raw_pointcloud, range_cropped_pointcloud, no_ground_pointcloud)
    occupancy_grid_map_nodes = pipeline.occupancy_grid_map(no_ground_pointcloud, range_cropped_pointcloud, grid_map, obstacle_segmentation_pointcloud)
    imu_nodes = pipeline.imu()

    nodes.extend(lidar_preprocess_nodes)
    nodes.extend(occupancy_grid_map_nodes)
    nodes.extend(imu_nodes)

    return nodes

def generate_launch_description():

    launch_arguments = []

    def add_launch_arg(name: str, default_value=None):
        launch_arguments.append(DeclareLaunchArgument(name, default_value=default_value))

    vehicle_info_param_path_default = os.path.join(
        get_package_share_directory('ichthus_launch'), 'param/vehicle_info.param.yaml'
    )

    voxel_grid_downsample_filter_param_path_default = os.path.join(
        get_package_share_directory('ichthus_launch'), 'param/voxel_grid_filter.param.yaml'
    )

    imu_corrector_param_path_default = os.path.join(
        get_package_share_directory('ichthus_launch'), 'param/imu_corrector.param.yaml'
    )

    add_launch_arg('raw_pointcloud_topic', '/lidar_front/points_raw'),
    add_launch_arg('no_ground_pointcloud_topic', '/lidar_front/no_ground_pointcloud'),
    add_launch_arg('range_cropped_pointcloud_topic', '/lidar_front/range_cropped_pointcloud')
    add_launch_arg('grid_map_topic', '/perception/occupancy_grid_map/map')
    add_launch_arg('obstacle_segmentation_pointcloud_topic', '/perception/obstacle_segmentation/pointcloud')
    add_launch_arg('vehicle_info_param_path', vehicle_info_param_path_default),
    add_launch_arg('use_sim_time', 'False')
    add_launch_arg('voxel_grid_downsample_filter_param_path', voxel_grid_downsample_filter_param_path_default)
    add_launch_arg('imu_corrector_param_path', imu_corrector_param_path_default)
    add_launch_arg('use_multi_lidar', 'False')

    return launch.LaunchDescription(
        launch_arguments 
        + [OpaqueFunction(function=launch_setup)]
    )