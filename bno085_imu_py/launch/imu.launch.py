from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Lade Parameter aus YAML-Datei
    config = os.path.join(
        get_package_share_directory('bno085_imu_py'),
        'config',
        'imu_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='bno085_imu_py',
            executable='bno085_imu_node',
            name='bno085_imu',
            parameters=[{
                'i2c_bus': 3,
                'i2c_address': '0x4B',
                'frame_id': 'imu_link',
                'pub_rate_hz': 50,
                'imu_topic': '/imu/data'
            }, config],  # Lade zusätzliche Parameter aus YAML
            output='screen',
            arguments=['--ros-args', '--log-level', 'DEBUG']
        )
    ])