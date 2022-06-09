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

class Map:
    def __init__(self, context):
        self.context = context
        self.lanelet2_map_name = LaunchConfiguration('lanelet2_map').perform(self.context).split('/')[-1]
        self.pointcloud_map_name = LaunchConfiguration('pointcloud_map').perform(self.context).split('/')[-1]
        
        self.map_odom_tf_dict = {
            'SanFrancisco.osm' : ["0", "0", "10.578049659729004", "0", "0", "0", "map", "odom"],
        }

        if self.map_odom_tf_dict.get(self.lanelet2_map_name) != None:
            self.map_odom_tf = self.map_odom_tf_dict[self.lanelet2_map_name]
        else:
            self.map_odom_tf = ["0", "0", "0", "0", "0", "0", "map", "odom"]

    def pointcloud_map_pipeline(self):
        pointcloud_map_loader = Node(
            package="map_loader",
            executable="pointcloud_map_loader",
            name="pointcloud_map_loader",
            remappings=[("output/pointcloud_map", "pointcloud_map")],
            parameters=[
                {
                    "pcd_paths_or_directory": [ LaunchConfiguration('pointcloud_map').perform(self.context)],
                    'use_sim_time' : LaunchConfiguration('use_sim_time'),
                }
            ],
            condition=IfCondition(LaunchConfiguration('use_pointcloud_map'))
        )

        map_hash_generator = Node(
            package="map_loader",
            executable="map_hash_generator",
            name="map_hash_generator",
            parameters=[
                {
                    "lanelet2_map_path": LaunchConfiguration('lanelet2_map'),
                    "pointcloud_map_path": LaunchConfiguration('pointcloud_map'),
                    'use_sim_time' : LaunchConfiguration('use_sim_time'),
                }
            ],
            condition=IfCondition(LaunchConfiguration('use_pointcloud_map'))
        )

        map_tf_generator = Node(
            package="map_tf_generator",
            executable="map_tf_generator",
            name="map_tf_generator",
            parameters=[
                {
                    "map_frame": "map",
                    "viewer_frame": "viewer",
                    'use_sim_time' : LaunchConfiguration('use_sim_time'),
                }
            ],
            condition=IfCondition(LaunchConfiguration('use_pointcloud_map'))
        )


        return [pointcloud_map_loader, map_tf_generator]

    def lanelet2_map_pipeline(self):

        lanelet2_map_loader_param_path = LaunchConfiguration('lanelet2_map_loader_param_path').perform(self.context)
        with open(lanelet2_map_loader_param_path, 'r') as f:
            lanelet2_map_loader_param = yaml.safe_load(f)["/**"]["ros__parameters"]
        lanelet2_map_loader_param = lanelet2_map_loader_param[self.lanelet2_map_name]
        
        lanelet2_map_loader = Node(
            package="map_loader",
            executable='lanelet2_map_loader',
            name="lanelet2_map_loader",
            remappings=[
                ("output/lanelet2_map", "/map/vector_map")
            ],
            parameters=[
                {
                    "center_line_resolution": 5.0,
                    "lanelet2_map_path": LaunchConfiguration('lanelet2_map'),
                    "lanelet2_map_projector_type": LaunchConfiguration('projector_type'),  # Options: MGRS, UTM
                    'use_sim_time' : LaunchConfiguration('use_sim_time')
                },
                lanelet2_map_loader_param
            ],
            condition=IfCondition(LaunchConfiguration('use_lanelet2_map'))
        )

        lanelet2_map_visualization = Node(
            package="map_loader",
            executable='lanelet2_map_visualization',
            name="lanelet2_map_visualization",
            remappings=[
                ("input/lanelet2_map", "/map/vector_map"),
                ("output/lanelet2_map_marker", "/map/vector_map_marker"),
            ],
            parameters=[
                {
                    'use_sim_time' : LaunchConfiguration('use_sim_time')
                }
            ],
            condition=IfCondition(LaunchConfiguration('use_lanelet2_map'))
        )

        map_odom_publisher = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=self.map_odom_tf,
            parameters=[
                {
                    'use_sim_time' : LaunchConfiguration('use_sim_time')
                }
            ],
            condition=UnlessCondition(LaunchConfiguration('use_pointcloud_map'))
        )
        ###only use when LGSVL Simulation without pointcloud map
        ###if Simulate only with lanelet2 map, fix the z value from the LGSVL 
        ###z is decide by the /lgsvl/gnss_odom z value
        ###if /lgsvl/gnss_odom x: 100, y:100, z:10.578049659729004, the z value of arguments is 10.578049659729004
        return [lanelet2_map_loader, lanelet2_map_visualization, map_odom_publisher]

    
def launch_setup(context, *args, **kwargs):
    pipeline = Map(context)

    nodes = list()
    
    pointcloud_map_nodes = pipeline.pointcloud_map_pipeline()
    lanelet2_map_nodes = pipeline.lanelet2_map_pipeline()

    nodes.extend(pointcloud_map_nodes)
    nodes.extend(lanelet2_map_nodes)

    return nodes


def generate_launch_description():

    launch_arguments = []

    def add_launch_arg(name: str, default_value=None):
        launch_arguments.append(DeclareLaunchArgument(name, default_value=default_value))

    lanelet2_map_loader_param_path_default = os.path.join(
        get_package_share_directory('ichthus_launch'), 'param/lanelet2_map_loader.param.yaml'
    )

    add_launch_arg('pointcloud_map', '')
    add_launch_arg('lanelet2_map', 'BorregasAve.osm')
    add_launch_arg('pointcloud_map','BorregasAve.pcd')
    add_launch_arg('use_pointcloud_map', 'False')
    add_launch_arg('use_lanelet2_map', 'False')
    add_launch_arg('lanelet2_map_loader_param_path', lanelet2_map_loader_param_path_default)
    add_launch_arg('use_sim_time', 'False')
    add_launch_arg('projector_type', 'UTM')
    return launch.LaunchDescription(
        launch_arguments 
        + [OpaqueFunction(function=launch_setup)]
    )