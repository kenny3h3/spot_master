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
                {'use_imu': True},
                {'imu_topic': '/bno085/imu'},
                {'output_mode': 'pwm'},
                {'front_bus': 6},
                {'back_bus': 4},
                {'front_state_topic': '/smov/front_state'},
                {'back_state_topic': '/smov/back_state'},
                {'proportional_scale': 0.5},
                {'channels_front': [0,1,2,3,4,5]},
                {'channels_back':  [0,1,2,3,4,5]},
            ]
        )
    ])
