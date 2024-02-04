from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('ohm_tsd_slam'),
        'config',
        'double-laser.yaml'
        )

    return LaunchDescription([
        Node(
            package='ohm_tsd_slam',
            executable='slam_node',
            name='tsd_multi_slam',
            remappings=[
                # Subscriptions
                ('/tsd_multi_slam/simon/laser', '/tsd_multi_slam/simon/laser'),
                ('/tsd_multi_slam/georg/laser', '/tsd_multi_slam/georg/laser'),
                # Publisher
                ('/tsd_multi_slam/map', 'tsd_multi_slam/map'),
                ('/tsd_multi_slam/simon/estimated_pose', 'tsd_multi_slam/simon/estimated_pose'),
                ('/tsd_multi_slam/georg/estimated_pose', 'tsd_multi_slam/georg/estimated_pose'),
                ('/tsd_multi_slam/map/image', 'tsd_multi_slam/map/image'),
                # Services
                ('/tsd_multi_slam/start_stop_slam', 'tsd_multi_slam/start_stop_slam'),
                ('/tsd_multi_slam/get_map', 'tsd_multi_slam/get_map')
            ],
            parameters=[config],
        )
    ])