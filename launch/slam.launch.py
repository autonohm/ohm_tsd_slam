from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('ohm_tsd_slam'),
        'config',
        'single-laser.yaml'
        )

    slam_node = Node(
        package='ohm_tsd_slam',
        executable='slam_node',
        name='tsd_slam',
        remappings=[
            # Subscriptions
            ('/tsd_slam/laser', '/scan'),
            # Publisher
            ('/tsd_slam/map', 'tsd_slam/map'),
            ('/tsd_slam/estimated_pose', 'tsd_slam/estimated_pose'),
            ('/tsd_slam/map/image', 'tsd_slam/map/image'),
            # Services
            ('/tsd_slam/start_stop_slam', 'tsd_slam/start_stop_slam'),
            ('/tsd_slam/get_map', 'tsd_slam/get_map')
        ],
        parameters=[config]
        # prefix=['gdbserver localhost:3000']
    )

    tf_laser_footprint = Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      arguments=[
        '0.3', '0', '0', '1.570796327', '0', '0',
        'base_footprint',
        'laser'
      ]
    )
    tf_footprint_odom = Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      arguments=[
        '1', '2', '0', '1', '0', '0',
        'odom',
        'base_footprint'
      ]
    )     

    return LaunchDescription([
        tf_laser_footprint,
        # tf_footprint_odom,
        slam_node
    ])
