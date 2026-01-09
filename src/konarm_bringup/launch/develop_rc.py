import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import subprocess


def generate_launch_description():

    config_konarm = PathJoinSubstitution([FindPackageShare("konarm_bringup"), "config", "konarm_driver_parameters.yaml"])

    return LaunchDescription([
        Node(
            package='konarm_driver',
            namespace='konarm',
            executable='konarm_driver',
            name='konarm_driver',
            parameters=[config_konarm]
        ),
        Node(
            package='konarm_driver',
            namespace='konarm',
            executable='konarm_basic_joystick_driver',
            name='konarm_basic_joystick_driver'
        ),
        Node(
            package='remote_6d',
            namespace='konarm',
            executable='remote_6d_node',
            name='remote_6d_node'
        ),
        # Node(
        #     package='sdrac_can_stranslator',
        #     namespace='sdrac',
        #     executable='can_code',
        #     name='can_code',
        #     parameters=[
        #         {"can_db_file": can_dbc_file}
        #     ]
        # ),
    ])
