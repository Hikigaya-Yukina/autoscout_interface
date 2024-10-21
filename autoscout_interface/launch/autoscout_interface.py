from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='autoscout_interface',
            executable='Auto22scout_main',
            name='autoscout_interface'
        )
    ])