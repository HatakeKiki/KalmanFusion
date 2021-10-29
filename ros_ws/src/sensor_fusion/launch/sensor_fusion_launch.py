from ament_index_python.packages import get_package_share_directory
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
import os.path

def generate_launch_description():
    rviz_config_dir = os.path.join(get_package_share_directory('sensor_fusion'), 'config', 'fusion.rviz')
    para_dir = os.path.join(get_package_share_directory('sensor_fusion'), 'config', 'sensor_fusion_subscribed_topic.yaml')
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'node_prefix',
            default_value=[launch.substitutions.EnvironmentVariable('USER'), '.'],
            description='Prefix for node names'
        ),
        launch_ros.actions.Node(
            package='rviz2',
            node_namespace='rviz2',
            node_executable='rviz2',
            arguments=['-d', rviz_config_dir]
        ),
        launch_ros.actions.Node(
            package='kitti_pub',
            node_namespace='kitti_pub',
            node_executable='kitti_pub',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='sensor_fusion',
            node_namespace='sensor_fusion',
            node_executable='sensor_fusion',
            parameters=[para_dir],
            output='screen'
        )
    ])
