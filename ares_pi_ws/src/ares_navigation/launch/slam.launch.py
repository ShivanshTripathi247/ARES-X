import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start SLAM Toolbox in Async mode (Best for live driving and mapping)
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                {'use_sim_time': False},
                {'odom_frame': 'odom'},
                {'map_frame': 'map'},
                {'base_frame': 'base_link'},
                {'scan_topic': '/scan'},
                {'mode': 'mapping'}
            ]
        )
    ])