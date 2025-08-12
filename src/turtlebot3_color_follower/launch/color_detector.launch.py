from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlebot3_color_follower',
            executable='color_detector',
            name='color_detector_node',
            output='screen',
            parameters=[{
                'image_topic': '/camera/image_raw',
                'cmd_vel_topic': '/cmd_vel',
                'linear_speed': 0.15,
                'angular_gain': 0.002,
                'center_tolerance': 30,
                'min_area': 500.0,
                'max_area': 8000.0,
                'smoothing_alpha': 0.6,
                'max_angular': 0.6,
                'max_linear': 0.25,
                'lost_timeout': 1.0,
                'show_debug_windows': True,   # <--- Add this line
            }]
        )
    ])
