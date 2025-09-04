from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Deklariere Launch-Argumente
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    log_level = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error)'
    )

    # Erstelle den Node
    spot_micro_motion_cmd_node = Node(
        package='spot_micro_motion_cmd',
        executable='spot_micro_motion_cmd',
        name='spot_micro_motion_cmd_node',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    )

    # Rückgabe der Launch-Beschreibung
    return LaunchDescription([
        use_sim_time,
        log_level,
        spot_micro_motion_cmd_node
    ])
