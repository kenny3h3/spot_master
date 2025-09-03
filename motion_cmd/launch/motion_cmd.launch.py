from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='motion_cmd',
            executable='motion_cmd_node',
            name='motion_cmd',
            output='screen',
            parameters=[{
                'update_rate_hz': 100.0,
                'gait_frequency_hz': 1.8,
                'step_height': 2.0,
                'neutral_y': 6.0,
                'neutral_x_front': 3.5,
                'neutral_x_back': 3.7,
                'max_stride_x': 2.0,
                'max_stride_y': 2.0,
                'upper_leg_length': 10.75,
                'lower_leg_length': 13.0,
                'hip_body_distance': 5.5,
            }],
        ),
    ])

