from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='controler',
            namespace='sdrac',
            executable='controler_translator',
            name='controler_translator'
        ),
        Node(
            package='controler',
            namespace='sdrac',
            executable='controler_rc',
            name='controler_rc'
        ),
        Node(
            package='rc_six_axis',
            namespace='sdrac',
            executable='rc_node',
            name='rc_node'
        ),
        Node(
            package='sdrac_can_stranslator',
            namespace='sdrac',
            executable='can_code',
            name='can_code'
        ),
    ])
