import os
from ament_index_python.packages import get_package_share_directory
import launch
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
# from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
# from launch.substitutions import PathJoinSubstitution
# from launch_ros.actions import ComposableNodeContainer
# from launch_ros.actions import LoadComposableNodes
# from launch_ros.descriptions import ComposableNode
# from launch_ros.substitutions import FindPackageShare
import yaml


class System:

    def __init__(self, context):
        self.context = context
        self.base_path = os.path.join(get_package_share_directory('ichthus_launch'), 'rviz/',)
        self.rviz_config_path = self.base_path + LaunchConfiguration('rviz_config_path').perform(self.context)
        self.rviz_splash_image_path = self.base_path + 'image/ichthus.jpg'
        self.urdf_file_path = LaunchConfiguration('urdf_file_path').perform(self.context)

    def system(self):

        rviz2 = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', self.rviz_config_path, '-s', self.rviz_splash_image_path],
            parameters=[
                {
                    'use_sim_time' : LaunchConfiguration('use_sim_time')
                }
            ],
            condition = IfCondition(LaunchConfiguration('rviz'))
        )

        with open(self.urdf_file_path, 'r') as infp:
            urdf_file = infp.read()
        
        urdf_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='urdf_publisher',
            parameters=[
                {'robot_description': urdf_file},
                {'use_sim_time' : LaunchConfiguration('use_sim_time')}
            ],
            output='screen'
        )

        ichthus_vehicle_interface = Node(
            package='ichthus_vehicle_interface',
            executable='ichthus_vehicle_interface_node',
            name='ichthus_vehicle_interface',
            remappings=[
                ('output/velocity_report', '/vehicle/status/velocity_status'),
                ('output/steering_report', '/vehicle/status/steering_status'),
                ('input/control_cmd', '/control/trajectory_follower/control_cmd')

            ],
            parameters=[
                {
                    'use_sim_time' : LaunchConfiguration('use_sim_time'),
                    'use_raw_odom' : LaunchConfiguration('use_raw_odom')
                }
            ],
            output='screen',
            condition=UnlessCondition(LaunchConfiguration('lgsvl'))
        )
        return [rviz2, urdf_publisher, ichthus_vehicle_interface]


def launch_setup(context, *args, **kwargs):
    pipeline = System(context)

    nodes = pipeline.system()

    return nodes


def generate_launch_description():

    launch_arguments = []

    def add_launch_arg(name: str, default_value=None):
        launch_arguments.append(DeclareLaunchArgument(name, default_value=default_value))

    urdf_file_path_default = os.path.join(
        get_package_share_directory('ichthus_launch'), 'urdf/lexus_rx_450h.urdf'
    )

    add_launch_arg('rviz_config_path', 'ichthus.rviz')
    add_launch_arg('urdf_file_path', urdf_file_path_default)
    add_launch_arg('rviz', 'true')
    add_launch_arg('use_sim_time', 'False')
    add_launch_arg('use_raw_odom', 'False')
    add_launch_arg('lgsvl', 'False')

    return launch.LaunchDescription(
        launch_arguments
        +[OpaqueFunction(function=launch_setup),
        # ExecuteProcess(
        #     cmd=[
        #         "ros2","topic","pub", "/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/path_change_approval", "tier4_planning_msgs/msg/Approval", 
        #         "{approval: true}", "-r", "1",
        #     ]
        # ),

        ExecuteProcess(
            cmd=[
                "ros2", "topic", "pub", "/autoware/engage", "autoware_auto_vehicle_msgs/msg/Engage",
                "{engage: true}","-r", "10", "-t", "100",
            ]
        )]
        
    )