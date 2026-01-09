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
import xacro
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    config_konarm = os.path.join(get_package_share_directory("konarm_bringup"), "config", "konarm_driver_parameters.yaml")

    robot_urdf = xacro.process_file(os.path.join(get_package_share_directory("konarm_bringup"), 'urdf', 'sdrac.urdf.xacro')).toxml()


    konarm_driver = Node(
            package='konarm_driver',
            executable='konarm_driver',
            name='konarm_driver',
            parameters=[config_konarm]
        )

    konarm_driver_basic =  Node(
            package='konarm_driver',
            executable='konarm_basic_joystick_driver',
            name='konarm_basic_joystick_driver'
        )

    remote_rc =   Node(
            package='remote_6d',
            executable='remote_6d_node',
            name='remote_6d_node'
        )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_urdf}
        ]
    )


    return LaunchDescription([
        robot_state_publisher_node,
        konarm_driver,
        konarm_driver_basic,
        remote_rc
    ])
