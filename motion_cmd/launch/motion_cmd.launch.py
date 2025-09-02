from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='motion_cmd',
            executable='motion_cmd_node',
            name='motion_cmd',
            output='screen',
            parameters=[
                {'update_rate_hz': 100.0},
                {'use_imu': False},
                {'imu_topic': '/imu/data'},
                {'output_mode': 'pwm'},
                {'front_bus': 1},
                {'back_bus': 6},
                {'front_state_topic': '/smov/front_state'},
                {'back_state_topic': '/smov/back_state'},
                {'proportional_scale': 0.5},
            ]
        )
    ])
