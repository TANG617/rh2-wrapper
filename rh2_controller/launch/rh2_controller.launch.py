from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rh2_controller',
            executable='rh2_dual_hand_node',
            name='rh2_dual_hand_node',
            output='screen',
            emulate_tty=True,
        )
    ])
