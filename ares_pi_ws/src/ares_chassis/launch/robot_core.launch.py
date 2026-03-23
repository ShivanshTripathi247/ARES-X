from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        
        # 1. THE CHASSIS: Motor Driver Node
        Node(
            package='ares_chassis',
            executable='motor_driver',
            name='motor_driver',
            output='screen'
        ),

        # 2. THE TF TREE: Base Link to Laser
        # This tells ROS: The LiDAR is 0.1 meters (10cm) forward and 0.15 meters up from the wheels.
        # (You can adjust the '0.1 0.0 0.15' numbers tomorrow based on a ruler measurement)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser',
            arguments=['0.1', '0.0', '0.15', '0.0', '0.0', '0.0', 'base_link', 'laser']
        ),

        # 3. THE SENSOR: RPLiDAR
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{'channel_type': 'serial',
                         'serial_port': '/dev/ttyUSB0', 
                         'serial_baudrate': 115200, 
                         'frame_id': 'laser',
                         'inverted': False, 
                         'angle_compensate': True}],
            output='screen'
        ),

        # -------------------------------------------------------------------
        # 4. THE VISION: Pi Cam V3 (libcamera)
        # FALLBACK PLAN: If using DroidCam instead, simply comment out this entire Node block.
        # -------------------------------------------------------------------
        Node(
            package='camera_ros',
            executable='camera_node',
            name='pi_cam_v3',
            parameters=[
                {'width': 640},
                {'height': 480},
                {'format': 'RGB888'}
            ],
            output='screen'
        )
    ])