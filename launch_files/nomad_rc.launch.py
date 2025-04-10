from launch import LaunchDescription
from launch_ros.actions import Node
import os

def resolve_relative_path(relative_path):
  return os.path.abspath(os.path.join(os.path.dirname(__file__), relative_path))



def generate_launch_description():
    can_dbc_file = resolve_relative_path("../src/sdrac_can_stranslator/sdrac_can_stranslator/ariadna_constants/can_messages/output/can.dbc")
    print(can_dbc_file)
    return LaunchDescription([
        Node(
            package='controler',
            namespace='sdrac',
            executable='controler_translator',
            name='controler_translator',
            parameters=[
                {"can_db_file" : can_dbc_file }
            ]
        ),
        Node(
            package='pilot_6axis',
            namespace='sdrac',
            executable='rc_node',
            name='rc_node'
        ),
        Node(
            package='sdrac_can_stranslator',
            namespace='sdrac',
            executable='can_code',
            name='can_code',
            parameters=[
              {"can_db_file": can_dbc_file}
            ]
        ),
    ])
