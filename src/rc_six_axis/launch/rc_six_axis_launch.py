import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='rc_six_axis',
            executable='rc_node',
            name='rc_node'),
  ])