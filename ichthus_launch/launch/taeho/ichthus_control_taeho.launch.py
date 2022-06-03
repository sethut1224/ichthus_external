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

class Control:

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

    def control(self):

        lat_controller_param_path = LaunchConfiguration('lat_controller_param_path').perform(self.context)
        lon_controller_param_path = LaunchConfiguration('lon_controller_param_path').perform(self.context)
        latlon_muxer_param_path = LaunchConfiguration('latlon_muxer_param_path').perform(self.context)
        vehicle_cmd_gate_param_path = LaunchConfiguration('vehicle_cmd_gate_param_path').perform(self.context)

        with open(lat_controller_param_path, "r") as f:
            lat_controller_param = yaml.safe_load(f)["/**"]["ros__parameters"]
        with open(lon_controller_param_path, "r") as f:
            lon_controller_param = yaml.safe_load(f)["/**"]["ros__parameters"]
        with open(latlon_muxer_param_path, "r") as f:
            latlon_muxer_param = yaml.safe_load(f)["/**"]["ros__parameters"]
        with open(vehicle_cmd_gate_param_path, "r") as f:
            vehicle_cmd_gate_param = yaml.safe_load(f)["/**"]["ros__parameters"]

        lat_controller = Node(
            package="trajectory_follower_nodes",
            executable="lateral_controller_node_exe",
            name="lateral_controller",
            remappings=[
                ("~/input/reference_trajectory", "/planning/scenario_planning/trajectory"),
                ("~/input/current_odometry", "/lgsvl/gnss_odom"),
                ("~/input/current_steering", "/vehicle/status/steering_status"),
                ("~/output/control_cmd", "lateral/control_cmd"),
                ("~/output/predicted_trajectory", "lateral/predicted_trajectory"),
                ("~/output/diagnostic", "lateral/diagnostic"),
            ],
            parameters=[
                lat_controller_param,
                self.vehicle_info,
                {
                'use_sim_time' : LaunchConfiguration('use_sim_time')
                }
            ],
        )

        lon_controller = Node(
            package="trajectory_follower_nodes",
            executable="longitudinal_controller_node_exe",
            name="longitudinal_controller",
            remappings=[
                ("~/input/current_trajectory", "/planning/scenario_planning/trajectory"),
                ("~/input/current_odometry", "/lgsvl/gnss_odom"),
                ("~/output/control_cmd", "longitudinal/control_cmd"),
                ("~/output/slope_angle", "longitudinal/slope_angle"),
            ("~/output/diagnostic", "longitudinal/diagnostic"),
        ],
            parameters=[
                lon_controller_param,
                self.vehicle_info,
                {
                    "show_debug_info": True,
                    "enable_pub_debug": True,
                    'use_sim_time' : LaunchConfiguration('use_sim_time')
                },
            ],
        )

        latlon_muxer= Node(
            package="trajectory_follower_nodes",
            executable="latlon_muxer_node_exe",
            name="latlon_muxer",
            remappings=[
                ("~/input/lateral/control_cmd", "lateral/control_cmd"),
                ("~/input/longitudinal/control_cmd", "longitudinal/control_cmd"),
                ("~/output/control_cmd", "control/trajectory_follower/control_cmd"),
            ],
            parameters=[
                latlon_muxer_param,
                {
                    'use_sim_time' : LaunchConfiguration('use_sim_time')
                }
            ],

        )
        vehicle_cmd_gate = Node(
            package="vehicle_cmd_gate",
            executable="vehicle_cmd_gate",
            name="vehicle_cmd_gate",
            remappings=[
                ("input/emergency_state", "/system/emergency/emergency_state"),
                ("input/steering", "/vehicle/status/steering_status"),
                ("input/auto/control_cmd", "/control/trajectory_follower/control_cmd"),
                ("input/auto/turn_indicators_cmd", "/planning/turn_indicators_cmd"),
                ("input/auto/hazard_lights_cmd", "/planning/hazard_lights_cmd"),
                ("input/auto/gear_cmd", "/control/shift_decider/gear_cmd"),
                ("input/external/control_cmd", "/external/selected/control_cmd"),
                ("input/external/turn_indicators_cmd", "/external/selected/turn_indicators_cmd"),
                ("input/external/hazard_lights_cmd", "/external/selected/hazard_lights_cmd"),
                ("input/external/gear_cmd", "/external/selected/gear_cmd"),
                ("input/external_emergency_stop_heartbeat", "/external/selected/heartbeat"),
                ("input/gate_mode", "/control/gate_mode_cmd"),
                ("input/emergency/control_cmd", "/system/emergency/control_cmd"),
                ("input/emergency/hazard_lights_cmd", "/system/emergency/hazard_lights_cmd"),
                ("input/emergency/gear_cmd", "/system/emergency/gear_cmd"),
                ("output/vehicle_cmd_emergency", "/control/command/emergency_cmd"),
                ("output/control_cmd", "/control/command/control_cmd"),
                ("output/gear_cmd", "/control/command/gear_cmd"),
                ("output/turn_indicators_cmd", "/control/command/turn_indicators_cmd"),
                ("output/hazard_lights_cmd", "/control/command/hazard_lights_cmd"),
                ("output/gate_mode", "/control/current_gate_mode"),
                ("output/engage", "/api/autoware/get/engage"),
                ("output/external_emergency", "/api/autoware/get/emergency"),
                ("~/service/engage", "/api/autoware/set/engage"),
                ("~/service/external_emergency", "/api/autoware/set/emergency"),
                # TODO(Takagi, Isamu): deprecated
                ("input/engage", "/autoware/engage"),
                ("~/service/external_emergency_stop", "~/external_emergency_stop"),
                ("~/service/clear_external_emergency_stop", "~/clear_external_emergency_stop"),
            ],
            parameters=[
                vehicle_cmd_gate_param,
                self.vehicle_info,
                {
                    "use_emergency_handling": False,
                    "use_external_emergency_stop": True,
                    "use_start_request": False,
                    'use_sim_time' : LaunchConfiguration('use_sim_time')
                },
            ],
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
            ]
            parameters=[
                {
                    'use_sim_time' : LaunchConfiguration('use_sim_time')
                }
            ]
        )

        return [lat_controller, lon_controller, latlon_muxer, vehicle_cmd_gate, state_report]

def launch_setup(context, *args, **kwargs):
    pipeline = Control(context)

    nodes = list()
    control_nodes = pipeline.control()
    nodes.extend(control_nodes)

    return nodes

def generate_launch_description():

    launch_arguments = []

    def add_launch_arg(name: str, default_value=None):
        launch_arguments.append(DeclareLaunchArgument(name, default_value=default_value))

    vehicle_info_param_path_default = os.path.join(
        get_package_share_directory('ichthus_launch'), 'param/vehicle_info.param.yaml'
    )

    lat_controller_param_path_default = os.path.join(
        get_package_share_directory('ichthus_launch'), 'param/lateral_controller.param.yaml'
    )

    lon_controller_param_path_default = os.path.join(
        get_package_share_directory('ichthus_launch'), 'param/longitudinal_controller.param.yaml'
    )

    latlon_muxer_param_path_default = os.path.join(
        get_package_share_directory('ichthus_launch'), 'param/latlon_muxer.param.yaml'
    )

    vehicle_cmd_gate_param_path_default = os.path.join(
        get_package_share_directory('ichthus_launch'), 'param/vehicle_cmd_gate.param.yaml'
    )

    add_launch_arg('vehicle_info_param_path', vehicle_info_param_path_default),
    add_launch_arg('use_sim_time', 'False')

    add_launch_arg('lat_controller_param_path', lat_controller_param_path_default)
    add_launch_arg('lon_controller_param_path', lon_controller_param_path_default)
    add_launch_arg('latlon_muxer_param_path', latlon_muxer_param_path_default)
    add_launch_arg('vehicle_cmd_gate_param_path', vehicle_cmd_gate_param_path_default)

    add_launch_arg('use_sim_time', 'False')
    return launch.LaunchDescription(
        launch_arguments 
        + [OpaqueFunction(function=launch_setup)]
    )
