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
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_base_to_velodyne',
            arguments=['0', '0', '0', '0', '0', '0', '/base_link', '/livox_frame'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_base_to_imu',
            arguments=['0.14', '0.02', '0.25', '0', '0', '-1.57', '/base_link', '/tugbot/imu_link/imu'],
        ),
    ])
