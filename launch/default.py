from launch import LaunchDescription
from launch_ros.actions import Node
import os

hostname = os.uname()[1].replace("-", "_")

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='home_assistant_bridge',
            namespace='',
            executable='home_assistant_bridge',
            name='home_assistant_bridge',
            parameters=[
                {"host": "192.168.1.100"},
            ],
        ),
    ])

