from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            output='screen'
        ),
        Node(
            package='turtlesim_color_change',
            executable='change_background',
            output='screen'
        ),
        Node(
            package='turtlesim',
            prefix = 'xterm -e',
            executable='turtle_teleop_key',
            output='screen'
        )
    ])
