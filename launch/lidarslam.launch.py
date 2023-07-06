import os
from launch.actions import DeclareLaunchArgument
import launch
import launch_ros.actions
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true'),
    
    main_param_dir = launch.substitutions.LaunchConfiguration(
        'main_param_dir',
        default=os.path.join(
            get_package_share_directory('tugbot_autoware_pkg'),
            'param',
            'lidarslam.yaml'))

    mapping = launch_ros.actions.Node(
        package='scanmatcher',
        executable='scanmatcher_node',
        parameters=[main_param_dir,],
        remappings=[('/input_cloud','/world/world_demo/model/tugbot/link/scan_omni/sensor/scan_omni/scan/points'),
                    ('/imu', '/world/world_demo/model/tugbot/link/imu_link/sensor/imu/imu')],
        output='screen'
        )
    
    graphbasedslam = launch_ros.actions.Node(
        package='graph_based_slam',
        executable='graph_based_slam_node',
        parameters=[main_param_dir],
        output='screen'
        )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'main_param_dir',
            default_value=main_param_dir,
            description='Full path to main parameter file to load'),
            mapping,
            graphbasedslam,
    ])