import os
from ament_index_python.packages import get_package_share_directory
import launch
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
# from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
# from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
# from launch.substitutions import PathJoinSubstitution
# from launch_ros.actions import ComposableNodeContainer
# from launch_ros.actions import LoadComposableNodes
# from launch_ros.descriptions import ComposableNode
# from launch_ros.substitutions import FindPackageShare
import yaml


class Planning:

    def __init__(self, context):
        self.context = context
        self.vehicle_info = self.get_vehicle_info()
        self.behavior_path_planner_param_directory_path = LaunchConfiguration('behavior_path_planner_param_directory_path').perform(self.context)

        self.side_shift_param_path = self.behavior_path_planner_param_directory_path + 'side_shift/side_shift.param.yaml'
        self.avoidance_param_path = self.behavior_path_planner_param_directory_path + 'avoidance/avoidance.param.yaml'
        self.lane_change_param_path = self.behavior_path_planner_param_directory_path + 'lane_change/lane_change.param.yaml'
        self.lane_following_param_path = self.behavior_path_planner_param_directory_path + 'lane_following/lane_following.param.yaml'
        self.pull_over_param_path = self.behavior_path_planner_param_directory_path + 'pull_over/pull_over.param.yaml'
        self.pull_out_param_path = self.behavior_path_planner_param_directory_path + 'pull_out/pull_out.param.yaml'
        self.behavior_path_planner_param_path = self.behavior_path_planner_param_directory_path + 'behavior_path_planner.param.yaml'
        self.bt_tree_config_param_path = self.behavior_path_planner_param_directory_path + LaunchConfiguration('behavior_tree').perform(self.context)

        self.behavior_velocity_planner_param_directory_path = LaunchConfiguration('behavior_velocity_planner_param_directory_path').perform(self.context)
        self.behavior_velocity_planner_param_path = self.behavior_velocity_planner_param_directory_path + 'behavior_velocity_planner.param.yaml'
        self.blind_spot_param_path = self.behavior_velocity_planner_param_directory_path + 'blind_spot.param.yaml'
        self.crosswalk_param_path = self.behavior_velocity_planner_param_directory_path + 'crosswalk.param.yaml'
        self.detection_area_param_path = self.behavior_velocity_planner_param_directory_path + 'detection_area.param.yaml'
        self.intersection_param_path = self.behavior_velocity_planner_param_directory_path + 'intersection.param.yaml'
        self.no_stopping_area_param_path = self.behavior_velocity_planner_param_directory_path + 'no_stopping_area.param.yaml'
        self.occlusion_spot_param_path = self.behavior_velocity_planner_param_directory_path + 'occlusion_spot.param.yaml'
        self.stop_line_param_path = self.behavior_velocity_planner_param_directory_path + 'stop_line.param.yaml'
        self.traffic_light_param_path = self.behavior_velocity_planner_param_directory_path + 'traffic_light.param.yaml'
        self.virtual_traffic_light_param_path = self.behavior_velocity_planner_param_directory_path + 'virtual_traffic_light.param.yaml'

        self.obstacle_planner_param_directory_path = LaunchConfiguration('obstacle_planner_param_directory_path').perform(self.context)
        self.obstacle_avoidance_planner_param_path = self.obstacle_planner_param_directory_path + 'obstacle_avoidance_planner/obstacle_avoidance_planner.param.yaml'
        self.obstacle_stop_planner_param_path =self.obstacle_planner_param_directory_path + 'obstacle_stop_planner/obstacle_stop_planner.param.yaml'
        self.adaptive_curise_control_param_path = self.obstacle_planner_param_directory_path + 'obstacle_stop_planner/adaptive_cruise_control.param.yaml'
        self.obstacle_cruise_planner_param_path = self.obstacle_planner_param_directory_path + 'obstacle_cruise_planner/obstacle_cruise_planner.param.yaml'

        self.common_param_path = LaunchConfiguration('common_param_path').perform(self.context)
        self.smoother_param_directory_path = LaunchConfiguration('smoother_param_directory_path').perform(self.context)
        self.smoother_type = LaunchConfiguration('smoother_type').perform(self.context)
        self.smoother_param_path = self.smoother_param_directory_path + self.smoother_type +'.param.yaml'
        self.motion_velocity_smoother_param_path = LaunchConfiguration('motion_velocity_smoother_param_path').perform(self.context)

        self.kinematic_state = LaunchConfiguration('kinematic_state').perform(self.context)

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

    def mission_planning(self):

        mission_planner = Node(
            package="mission_planner",
            executable='mission_planner',
            name="mission_planner",
            remappings=[
                ("input/vector_map", "/map/vector_map"),
                ("input/goal_pose", "/planning/mission_planning/goal"),
                ("input/checkpoint", "/planning/mission_planning/checkpoint"),
                ("output/route", "/planning/mission_planning/route"),
                ("debug/route_marker", "/planning/mission_planning/route_marker"),
            ],
            parameters=[
                {
                    "map_frame": "map",
                    "base_link_frame": "base_link",
                    'use_sim_time' : LaunchConfiguration('use_sim_time')
                }
            ],
        )

        goal_pose_visualizer = Node(
            package="mission_planner",
            executable="goal_pose_visualizer",
            name="goal_pose_visualizer",
            remappings=[
                ("input/route", "/planning/mission_planning/route"),
                ("output/goal_pose", "/planning/mission_planning/echo_back_goal_pose"),
            ],
            parameters=[
                {
                    'use_sim_time' : LaunchConfiguration('use_sim_time')
                }
            ]
        )

        return [mission_planner, goal_pose_visualizer]

    def behavior_planning(self):
        
        with open(self.side_shift_param_path, "r") as f:
            side_shift_param = yaml.safe_load(f)["/**"]["ros__parameters"]
        with open(self.avoidance_param_path, "r") as f:
            avoidance_param = yaml.safe_load(f)["/**"]["ros__parameters"]
        with open(self.lane_change_param_path, "r") as f:
            lane_change_param = yaml.safe_load(f)["/**"]["ros__parameters"]
        with open(self.lane_following_param_path, "r") as f:
            lane_following_param = yaml.safe_load(f)["/**"]["ros__parameters"]
        with open(self.pull_over_param_path, "r") as f:
            pull_over_param = yaml.safe_load(f)["/**"]["ros__parameters"]
        with open(self.pull_out_param_path, "r") as f:
            pull_out_param = yaml.safe_load(f)["/**"]["ros__parameters"]
        with open(self.behavior_path_planner_param_path, "r") as f:
            behavior_path_planner_param = yaml.safe_load(f)["/**"]["ros__parameters"]

        behavior_path_planner = Node(
            package="behavior_path_planner",
            executable="behavior_path_planner",
            name="behavior_path_planner",
            remappings=[
                ("~/input/route", '/planning/mission_planning/route'),
                ("~/input/vector_map", "/map/vector_map"),
                ("~/input/perception", "/perception/object_recognition/objects"),
                ("~/input/odometry", self.kinematic_state),
                ("~/input/scenario", "/planning/scenario_planning/scenario"),
                (
                    "~/input/external_approval",
                    "/planning/scenario_planning/lane_driving/behavior_planning/"
                    "behavior_path_planner/path_change_approval",
                ),
                (
                    "~/input/force_approval",
                    "/planning/scenario_planning/lane_driving/behavior_planning/"
                    "behavior_path_planner/path_change_force",
                ),
                ("~/output/path", "path_with_lane_id"),
                (
                    "~/output/ready",
                    "/planning/scenario_planning/lane_driving/behavior_planning/"
                    "behavior_path_planner/ready_module",
                ),
                (
                    "~/output/running",
                    "/planning/scenario_planning/lane_driving/behavior_planning/"
                    "behavior_path_planner/running_modules",
                ),
                (
                    "~/output/force_available",
                    "/planning/scenario_planning/lane_driving/behavior_planning/"
                    "behavior_path_planner/force_available",
                ),
                ("~/output/turn_indicators_cmd", "/planning/turn_indicators_cmd"),
                ("~/output/hazard_lights_cmd", "/planning/hazard_lights_cmd"),
            ],
            parameters=[
                side_shift_param,
                avoidance_param,
                lane_change_param,
                lane_following_param,
                pull_over_param,
                pull_out_param,
                behavior_path_planner_param,
                self.vehicle_info,
                {
                    "bt_tree_config_path":  self.bt_tree_config_param_path,
                    "planning_hz" : 10.0,
                    # 'backward_path_length' : 5.0,
                    # 'drivable_lane_backward_length':  5.0,
                    # 'drivable_lane_margin': 3.0,
                    'lane_change.enable_collision_check_at_prepare_phase': True,
                    'lane_change.lane_change_prepare_duration' : 3.0,
                    'lane_change.use_predicted_path_outside_lanelet': False,
                    'lane_change.use_all_predicted_path': False,
                    'use_sim_time' : LaunchConfiguration('use_sim_time')
                }
            ],
        )

        scenario_selector = Node(
            package="scenario_selector",
            executable='scenario_selector',
            name='scenario_selector',
            remappings=[
                ('input/lane_driving/trajectory', '/planning/scenario_planning/lane_driving/trajectory'),
                ('input/parking/trajectory', '/planning/scenario_planning/parking/trajectory'),
                ('input/lanelet_map', '/map/vector_map'),
                ('input/route', '/planning/mission_planning/route'),
                ('input/odometry', self.kinematic_state),
                ('is_parking_completed', '/planning/scenario_planning/parking/is_completed'),
                ('output/scenario' , '/planning/scenario_planning/scenario'),
                ('output/trajectory', '/planning/scenario_planning/scenario_selector/trajectory'),
            ],

            parameters=[
                {
                    'update_rate' : 10.0,
                    'th_max_message_delay_sec' : 1.0,
                    'th_arrived_distance_m' : 1.0,
                    'th_stopped_time_sec' : 1.0,
                    'th_stopped_velocity_mps' : 0.01,
                    'use_sim_time' : LaunchConfiguration('use_sim_time')
                }
            ]
        )

        with open(self.blind_spot_param_path, "r") as f:
            blind_spot_param = yaml.safe_load(f)["/**"]["ros__parameters"]
        with open(self.crosswalk_param_path, "r") as f:
            crosswalk_param = yaml.safe_load(f)["/**"]["ros__parameters"]
        with open(self.detection_area_param_path, "r") as f:
            detection_area_param = yaml.safe_load(f)["/**"]["ros__parameters"]
        with open(self.intersection_param_path, "r") as f:
            intersection_param = yaml.safe_load(f)["/**"]["ros__parameters"]
        with open(self.no_stopping_area_param_path, "r") as f:
            no_stopping_area_param = yaml.safe_load(f)["/**"]["ros__parameters"]
        with open(self.occlusion_spot_param_path, "r") as f:
            occlusion_spot_param = yaml.safe_load(f)["/**"]["ros__parameters"]
        with open(self.stop_line_param_path, "r") as f:
            stop_line_param = yaml.safe_load(f)["/**"]["ros__parameters"]
        with open(self.traffic_light_param_path, "r") as f:
            traffic_light_param = yaml.safe_load(f)["/**"]["ros__parameters"]
        with open(self.virtual_traffic_light_param_path, "r") as f:
            virtual_traffic_light_param = yaml.safe_load(f)["/**"]["ros__parameters"]
        with open(self.behavior_velocity_planner_param_path, "r") as f:
            behavior_velocity_planner_param = yaml.safe_load(f)["/**"]["ros__parameters"]

        behavior_velocity_planner = Node(
            package="behavior_velocity_planner",
            executable="behavior_velocity_planner_node",
            name="behavior_velocity_planner",
            remappings=[
                ("~/input/path_with_lane_id", "path_with_lane_id"),
                ("~/input/vector_map", "/map/vector_map"),
                ("~/input/vehicle_odometry", self.kinematic_state),
                ("~/input/dynamic_objects", "/perception/object_recognition/objects"),
                (
                    "~/input/no_ground_pointcloud",
                    "/perception/obstacle_segmentation/pointcloud",
                ),
                (
                    "~/input/traffic_signals",
                    "/perception/traffic_light_recognition/traffic_signals",
                ),
                (
                    "~/input/external_traffic_signals",
                    "/external/traffic_light_recognition/traffic_signals",
                ),
                ("~/input/virtual_traffic_light_states", "/awapi/tmp/virtual_traffic_light_states"),
                (
                    "~/input/occupancy_grid",
                    "/perception/occupancy_grid_map/map",
                ),
                ("~/input/external_velocity_limit_mps", '/planning/scenario_planning/max_velocity'),
                ("~/output/path", "/planning/scenario_planning/lane_driving/behavior_planning/path"),
                ("~/output/stop_reasons", "/planning/scenario_planning/status/stop_reasons"),
                (
                    "~/output/infrastructure_commands",
                    "/planning/scenario_planning/status/infrastructure_commands",
                ),
                ("~/output/traffic_signal", "debug/traffic_signal"),
            ],
            parameters=[
                behavior_velocity_planner_param,
                blind_spot_param,
                crosswalk_param,
                detection_area_param,
                intersection_param,
                stop_line_param,
                traffic_light_param,
                virtual_traffic_light_param,
                occlusion_spot_param,
                no_stopping_area_param,
                self.vehicle_info,
                {
                    # "backward_path_length": 1.0,
                    "delay_response_time": 0.01,
                    'use_sim_time' : LaunchConfiguration('use_sim_time')
                },
            ],
        )


        return [behavior_path_planner, scenario_selector, behavior_velocity_planner]

    def motion_planning(self):

        with open(self.obstacle_avoidance_planner_param_path, "r") as f:
            obstacle_avoidance_planner_param = yaml.safe_load(f)["/**"]["ros__parameters"]
        with open(self.obstacle_stop_planner_param_path, "r") as f:
            obstacle_stop_planner_param = yaml.safe_load(f)["/**"]["ros__parameters"]
        with open(self.adaptive_curise_control_param_path, "r") as f:
            adaptive_curise_control_param = yaml.safe_load(f)["/**"]["ros__parameters"]
        with open(self.obstacle_cruise_planner_param_path, "r") as f:
            obstacle_cruise_planner_param = yaml.safe_load(f)["/**"]["ros__parameters"]
        with open(self.common_param_path, "r") as f:
            common_param = yaml.safe_load(f)["/**"]["ros__parameters"]
        with open(self.motion_velocity_smoother_param_path, "r") as f:
            motion_velocity_smoother_param = yaml.safe_load(f)["/**"]["ros__parameters"]
        with open(self.smoother_param_path, "r") as f:
            smoother_param = yaml.safe_load(f)["/**"]["ros__parameters"]

        obstacle_avoidance_planner= Node(
            package="obstacle_avoidance_planner",
            executable="obstacle_avoidance_planner_node",
            name="obstacle_avoidance_planner",
            remappings=[
                ("~/input/objects", "/perception/object_recognition/objects"),
                ('localization/kinematic_state', self.kinematic_state),
                ("~/input/path", '/planning/scenario_planning/lane_driving/behavior_planning/path'),
                ("~/output/path", "obstacle_avoidance_planner/trajectory"),
            ],
            parameters=[
                obstacle_avoidance_planner_param,
                self.vehicle_info,
                {"is_showing_debug_info": True},
                {"is_stopping_if_outside_drivable_area": True},
                {'use_sim_time' : LaunchConfiguration('use_sim_time')}
            ],
        )

        obstacle_stop_planner = Node(
            package="obstacle_stop_planner",
            executable="obstacle_stop_planner_node",
            name="obstacle_stop_planner",
            namespace="",
            remappings=[
                ("~/output/stop_reason", "/planning/scenario_planning/status/stop_reason"),
                ("~/output/stop_reasons", "/planning/scenario_planning/status/stop_reasons"),
                ("~/output/max_velocity", "/planning/scenario_planning/max_velocity_candidates"),
                (
                    "~/output/velocity_limit_clear_command",
                    "/planning/scenario_planning/clear_velocity_limit",
                ),
                ("~/output/trajectory", "/planning/scenario_planning/lane_driving/trajectory"),
                (
                    "~/input/pointcloud",
                    "/perception/obstacle_segmentation/pointcloud",
                ),
                ("~/input/objects", "/perception/object_recognition/objects"),
                ("~/input/odometry", self.kinematic_state),
                ("~/input/trajectory", "obstacle_avoidance_planner/trajectory"),
            ],
            parameters=[
                common_param,
                obstacle_stop_planner_param,
                adaptive_curise_control_param,
                self.vehicle_info,
                {"enable_slow_down": False},
                {'use_sim_time' : LaunchConfiguration('use_sim_time')}
            ],
        )

        obstacle_cruise_planner = Node(
            package="obstacle_cruise_planner",
            executable="obstacle_cruise_planner",
            name="obstacle_cruise_planner",
            remappings=[
                ("~/input/trajectory", "obstacle_avoidance_planner/trajectory"),
                ("~/input/odometry", self.kinematic_state),
                ("~/input/objects", "/perception/object_recognition/objects"),
                ("~/output/trajectory", "/planning/scenario_planning/lane_driving/trajectory"),
                ("~/output/velocity_limit", "/planning/scenario_planning/max_velocity_candidates"),
                ("~/output/clear_velocity_limit", "/planning/scenario_planning/clear_velocity_limit"),
                ("~/output/stop_reasons", "/planning/scenario_planning/status/stop_reasons"),
            ],
            parameters=[
                common_param,
                self.vehicle_info,
                obstacle_cruise_planner_param,
                {
                    'use_sim_time' : LaunchConfiguration('use_sim_time'),
                    'is_showing_debug_info' : False
                }
            ],
            output='log'
        )

        external_velocity_limit_selector = Node(
            package='external_velocity_limit_selector',
            executable='external_velocity_limit_selector',
            name='external_velocity_limit_selector',
            remappings=[
                ("input/velocity_limit_from_api", "/planning/scenario_planning/max_velocity_default"),
                ("input/velocity_limit_from_internal" , "/planning/scenario_planning/max_velocity_candidates"),
                ("input/velocity_limit_clear_command_from_internal", "/planning/scenario_planning/clear_velocity_limit"),
                ("output/external_velocity_limit", '/planning/scenario_planning/max_velocity')
            ],
            parameters=[
                {
                    'param_path' : motion_velocity_smoother_param,
                    'common_param_path' : common_param,
                    'use_sim_time' : LaunchConfiguration('use_sim_time')
                }
            ]
        )

        motion_velocity_smoother = Node(
            package='motion_velocity_smoother',
            executable='motion_velocity_smoother',
            name='motion_velocity_smoother',
            remappings=[
                ("~/input/trajectory", '/planning/scenario_planning/scenario_selector/trajectory'),
                ("~/output/trajectory", '/planning/scenario_planning/trajectory'),
                ("~/input/external_velocity_limit_mps", "/planning/scenario_planning/max_velocity"),
                ("~/output/current_velocity_limit_mps", '/planning/scenario_planning/current_max_velocity'),
                ("/localization/kinematic_state", self.kinematic_state)
            ],
            parameters=[
                {
                    "smoother_type" : self.smoother_type,
                    'publish_debug_trajs' : True,
                    'algorithm_type' : self.smoother_type,
                    'param_path' : motion_velocity_smoother_param,
                    'common_param_path' : common_param,
                    'smoother_param_path' : smoother_param,
                    'use_sim_time' : LaunchConfiguration('use_sim_time'),
                    'max_velocity' : 50.0
                }
            ]
        )



        return [obstacle_avoidance_planner, obstacle_cruise_planner, external_velocity_limit_selector, motion_velocity_smoother]

def launch_setup(context, *args, **kwargs):
    pipeline = Planning(context)

    nodes = list()

    mission_planning_nodes = pipeline.mission_planning()
    behavior_planning_nodes = pipeline.behavior_planning()
    motion_planning_nodes = pipeline.motion_planning()

    nodes.extend(mission_planning_nodes)
    nodes.extend(behavior_planning_nodes)
    nodes.extend(motion_planning_nodes)

    return nodes


def generate_launch_description():

    launch_arguments = []

    def add_launch_arg(name: str, default_value=None):
        launch_arguments.append(DeclareLaunchArgument(name, default_value=default_value))

    vehicle_info_param_path_default = os.path.join(
        get_package_share_directory('ichthus_launch'), 'param/vehicle_info.param.yaml'
    )

    behavior_path_planner_param_directory_path_deafult = os.path.join(
        get_package_share_directory('ichthus_launch'), 'param/lane_driving/behavior_planning/behavior_path_planner/'
    )
    
    behavior_velocity_planner_param_directory_path_default = os.path.join(
        get_package_share_directory('ichthus_launch'), 'param/lane_driving/behavior_planning/behavior_velocity_planner/'
    )

    obstacle_planner_param_directory_path_default = os.path.join(
        get_package_share_directory('ichthus_launch'), 'param/lane_driving/motion_planning/'
    )
    
    common_param_path_default = os.path.join(
        get_package_share_directory('ichthus_launch'), 'param/common/common.param.yaml'
    )

    motion_velocity_smoother_param_path_default = os.path.join(
        get_package_share_directory('ichthus_launch'), 'param/common/motion_velocity_smoother/motion_velocity_smoother.param.yaml'
    )

    smoother_param_directory_path_default = os.path.join(
        get_package_share_directory('ichthus_launch'), 'param/common/motion_velocity_smoother/'
    )

    add_launch_arg('vehicle_info_param_path', vehicle_info_param_path_default)
    add_launch_arg('behavior_path_planner_param_directory_path', behavior_path_planner_param_directory_path_deafult)
    add_launch_arg('behavior_tree', 'behavior_path_planner_tree.xml')
    add_launch_arg('behavior_velocity_planner_param_directory_path', behavior_velocity_planner_param_directory_path_default)
    add_launch_arg('obstacle_planner_param_directory_path', obstacle_planner_param_directory_path_default)
    add_launch_arg('common_param_path', common_param_path_default)
    add_launch_arg('motion_velocity_smoother_param_path', motion_velocity_smoother_param_path_default)
    add_launch_arg('smoother_param_directory_path', smoother_param_directory_path_default)
    add_launch_arg('smoother_type', 'JerkFiltered')

    add_launch_arg('use_sim_time', 'False')
    add_launch_arg("kinematic_state", 'lgsvl/gnss_odom')

    return launch.LaunchDescription(
        launch_arguments
        +[OpaqueFunction(function=launch_setup)]
    )