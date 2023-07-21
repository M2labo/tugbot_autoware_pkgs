from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tugbot_autoware_pkg',
            executable='ackermann_to_twist_node',
            name='ackermann_to_twist',
        ),
        Node(
            package='tugbot_autoware_pkg',
            executable='steering_report_publisher_node',
            name='steering_report_publisher',
        ),
    ])
