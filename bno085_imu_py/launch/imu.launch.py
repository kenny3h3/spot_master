from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bno085_imu_py',
            executable='bno085_imu_node',
            name='bno085_imu',
            parameters=[{
                'i2c_bus': 3,
                'i2c_address': 0x4B,
                'frame_id': 'imu_link',
                'pub_rate_hz': 50,
            }],
            output='screen'
        )
    ])
