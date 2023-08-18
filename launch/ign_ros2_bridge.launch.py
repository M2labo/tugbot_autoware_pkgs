from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation (Gazebo) clock if true'),

        Node(
            package='ros_ign_bridge',
            executable='parameter_bridge',
            name='bridge_node',
            arguments=[
                '/model/tugbot/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
                '/world/world_demo/model/tugbot/link/scan_omni/sensor/scan_omni/scan/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',
                '/world/world_demo/model/tugbot/link/camera_front/sensor/color/image@sensor_msgs/msg/Image@ignition.msgs.Image',
                #'/model/tugbot/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
                '/model/tugbot/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
                '/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock',
                '/world/world_demo/model/tugbot/link/imu_link/sensor/imu/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU'
            ],
            remappings=[
                ('/model/tugbot/tf', '/tf'),
                #('/model/tugbot/odometry', '/localization/kinematic_state'),
            ],
            output='screen',
            parameters=[
                {"use_sim_time": LaunchConfiguration('use_sim_time')}
            ],
        ),
    ])