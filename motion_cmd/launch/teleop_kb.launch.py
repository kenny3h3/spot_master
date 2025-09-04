from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='motion_cmd',
            executable='teleop_kb',
            name='teleop_kb',
            output='screen',
            parameters=[{
                'rate_hz': 50.0,
                'linear_speed': 0.6,
                'angular_speed': 0.8,
                'accel': 3.0,
                'decel': 4.0,
            }],
        ),
    ])
