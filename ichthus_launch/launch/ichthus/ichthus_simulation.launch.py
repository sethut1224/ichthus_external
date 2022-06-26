# Copyright 2021 The Autoware Foundation.
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

import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml
import os
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    # vehicle information param path
    vehicle_info_param_path = LaunchConfiguration("vehicle_info_param_file").perform(context)
    with open(vehicle_info_param_path, "r") as f:
        vehicle_info_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    vehicle_characteristics_param_path = LaunchConfiguration(
        "vehicle_characteristics_param_file"
    ).perform(context)
    with open(vehicle_characteristics_param_path, "r") as f:
        vehicle_characteristics_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    simulator_model_param_path = LaunchConfiguration("simulator_model_param_file").perform(context)
    with open(simulator_model_param_path, "r") as f:
        simulator_model_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    simple_planning_simulator_node = Node(
        package="simple_planning_simulator",
        executable="simple_planning_simulator_exe",
        name="simple_planning_simulator",
        output="screen",
        parameters=[
            vehicle_info_param,
            vehicle_characteristics_param,
            simulator_model_param,
            {
                "initial_engage_state": LaunchConfiguration("initial_engage_state"),
            },
        ],
        remappings=[
            ("input/ackermann_control_command", "/control/command/control_cmd"),
            ("input/gear_command", "/control/command/gear_cmd"),
            ("input/turn_indicators_command", "/control/command/turn_indicators_cmd"),
            ("input/hazard_lights_command", "/control/command/hazard_lights_cmd"),
            ("input/trajectory", "/planning/scenario_planning/trajectory"),
            ("input/engage", "/vehicle/engage"),
            ("output/twist", "/vehicle/status/velocity_status"),
            ("output/odometry", "/localization/kinematic_state"),
            ("output/steering", "/vehicle/status/steering_status"),
            ("output/gear_report", "/vehicle/status/gear_status"),
            ("output/turn_indicators_report", "/vehicle/status/turn_indicators_status"),
            ("output/hazard_lights_report", "/vehicle/status/hazard_lights_status"),
            ("output/control_mode_report", "/vehicle/status/control_mode"),
            ("/initialpose", "/initialpose"),
        ],
    )

    map_to_odom_tf_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_map_to_odom_tf_publisher",
        output="screen",
        arguments=["0.0", "0.0", "0.0", "0", "0", "0", "map", "odom"],
    )

    group = GroupAction([simple_planning_simulator_node])

    return [group]


def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None, description=None):
        launch_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value, description=description)
        )
    
    vehicle_info_param_path_default = os.path.join(
        get_package_share_directory('ichthus_launch'), 'param/ichthus_vehicle_info.param.yaml'
    )

    add_launch_arg(
        "vehicle_info_param_file",
        vehicle_info_param_path_default,
        "path to the parameter file of vehicle information",
    )

    add_launch_arg(
        "vehicle_characteristics_param_file",
        [
            FindPackageShare("ichthus_launch"),
            "/param/vehicle_characteristics.param.yaml",
        ],
        "path to config file for vehicle characteristics",
    )

    add_launch_arg(
        "simulator_model_param_file",
        [
            FindPackageShare("ichthus_launch"),
            "/param/simple_planning_simulator_default.param.yaml",
        ],
        "path to config file for simulator_model",
    )

    return launch.LaunchDescription(launch_arguments + [OpaqueFunction(function=launch_setup)])
