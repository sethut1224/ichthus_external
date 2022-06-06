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

class LGSVL:
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
    
    def lgsvl(self):

        # urdf_path = LaunchConfiguration('urdf_path').perform(self.context)
        # with open(urdf_path, 'r') as infp:
        #     urdf_file = infp.read()
        
        # urdf_publisher = Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     name='urdf_publisher',
        #     parameters=[
        #         {
        #             'robot_description': urdf_file,
        #             'use_sim_time' : LaunchConfiguration('use_sim_time')
        #         }
        #     ],
        # )

        lgsv_interface_param_path = LaunchConfiguration('lgsvl_interface_param_path').perform(self.context)
        with open(lgsv_interface_param_path, 'r') as f:
            lgsv_interface_param = yaml.safe_load(f)["/**"]["ros__parameters"]

        lgsvl_interface = Node(
            package='lgsvl_interface',
            executable='lgsvl_interface_exe',
            name='lgsvl_interface',
            output='screen',
            parameters=[
              lgsv_interface_param,
              {"lgsvl.publish_tf": False},
              {"lgsvl.publish_pose": False},
              {'use_sim_time' : LaunchConfiguration('use_sim_time')}
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

        state_report = Node(
            package='state_report',
            executable='state_report_node',
            name='state_report',
            remappings=[
                ('input/vehicle_kinematic_state', '/vehicle/vehicle_kinematic_state'),
                ('input/turn_indicators_command', '/control/command/turn_indicators_cmd'),
                ('output/vehicle_velocity_report', '/vehicle/status/velocity_status'),
                ('output/turn_indicators_report', '/vehicle/status/turn_indicators_status'),
                ('output/steering_report', '/vehicle/status/steering_status')
            ],
            parameters=[
                {
                    'use_sim_time' : LaunchConfiguration('use_sim_time')
                }
            ]
        )

        return [lgsvl_interface, state_report]

    
def launch_setup(context, *args, **kwargs):
    pipeline = LGSVL(context)
    nodes = pipeline.lgsvl()

    return nodes

def generate_launch_description():
    launch_arguments = []
    def add_launch_arg(name: str, default_value=None):
        launch_arguments.append(DeclareLaunchArgument(name, default_value=default_value))

    vehicle_info_param_path_default = os.path.join(
        get_package_share_directory('ichthus_launch'), 'param/vehicle_info.param.yaml'
    )

    # urdf_path_default = os.path.join(
    #     get_package_share_directory('ichthus_launch'), 'urdf/lexus_rx_450h.urdf'
    # )

    lgsvl_interface_param_path_default = os.path.join(
        get_package_share_directory('ichthus_launch'), 'param/lgsvl_interface.param.yaml'
    )

    add_launch_arg('vehicle_info_param_path', vehicle_info_param_path_default)
    add_launch_arg('lgsvl_interface_param_path', lgsvl_interface_param_path_default)
    # add_launch_arg('urdf_path', urdf_path_default)
    add_launch_arg('use_sim_time', 'False')

    return launch.LaunchDescription(
        launch_arguments 
        + [OpaqueFunction(function=launch_setup)]
    )

 