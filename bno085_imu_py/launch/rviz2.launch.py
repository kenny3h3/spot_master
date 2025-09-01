from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    config_file = os.path.join(
        os.path.expanduser('~'),
        'smov', 'src', 'bno085_imu_py', 'launch', 'imu_rviz_config.rviz'
    )

    return LaunchDescription([
        # Dein IMU Node
        Node(
            package='bno085_imu_py',
            executable='bno085_imu_node',
            name='bno085_imu',
            output='screen',
            parameters=[{
                'i2c_bus': 3,
                'i2c_address': 0x4B,
                'frame_id': 'imu_link',
                'pub_rate_hz': 50,
            }]
        ),

        # RViz2 starten
        ExecuteProcess(
            cmd=['rviz2', '-d', config_file],
            output='screen'
        )
    ])
