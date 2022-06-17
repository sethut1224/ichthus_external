import os

import launch
from launch import LaunchDescription
from launch.actions import GroupAction
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import PushRosNamespace
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.actions import OpaqueFunction

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node

import ament_index_python

import yaml


def launch_setup(context, *args, **kwargs):

  g2m_poser = Node(
    package='g2m_poser',
    executable='g2m_poser_exe',
    name='g2m_poser',
    parameters=
    [
      {
        "use_sim_time": LaunchConfiguration("use_sim_time"),
        
        "use_tf_publish": LaunchConfiguration("use_tf_publish"),
        "target_frame_id": LaunchConfiguration("target_frame_id"),
        "pose_diff_for_heading": LaunchConfiguration("pose_diff_for_heading"),
        "origin_x": LaunchConfiguration("origin_x"),
        "origin_y": LaunchConfiguration("origin_y"),
        "origin_z": LaunchConfiguration("origin_z"),
        "heading_source": LaunchConfiguration("heading_source"),
        "fix_topic": LaunchConfiguration("fix_topic"),
        "imu_topic": LaunchConfiguration("imu_topic"),
        "odom_topic": LaunchConfiguration("odom_topic"),

        "pose_topic": LaunchConfiguration("pose_topic"),
        "pose_cov_topic": LaunchConfiguration("pose_cov_topic"),
      }
    ],
    output="screen",
    # extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
  )

  return [g2m_poser]


def generate_launch_description():
  """Generate launch description with multiple components."""
  launch_arguments = []
  def add_launch_arg(name: str, default_value=None, description=None):
    launch_arguments.append(
      DeclareLaunchArgument(name, default_value=default_value, description=description)
    )

  # component
  add_launch_arg("use_tf_publish", "false")
  add_launch_arg("target_frame_id", "gnss")
  
  add_launch_arg("origin_x", "'445815.539508'")
  add_launch_arg("origin_y", "'3944953.128090'")
  add_launch_arg("origin_z", "'48.640911'")

  add_launch_arg("heading_source", "NONE") # NONE, IMU, ODOM

  add_launch_arg("pose_diff_for_heading", "0.4")

  add_launch_arg("fix_topic", "/fix")
  add_launch_arg("imu_topic", "/imu/data")
  add_launch_arg("odom_topic", "/output/velocity_report")
  
  add_launch_arg("pose_topic", "/gnss_pose")
  add_launch_arg("pose_cov_topic", "/gnss_pose_cov")

  add_launch_arg("use_sim_time", "true")

  return launch.LaunchDescription(
    launch_arguments
    + [OpaqueFunction(function=launch_setup)]
  )
