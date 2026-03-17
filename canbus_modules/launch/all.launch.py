from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_power = PathJoinSubstitution([
        FindPackageShare('canbus_modules'), 'config', 'power_measurement.yaml'
    ])
    config_lamps = PathJoinSubstitution([
        FindPackageShare('canbus_modules'), 'config', 'lamps.yaml'
    ])

    return LaunchDescription([
        Node(
            package='canbus_modules',
            executable='power_measurement.py',
            name='power_measurement',
            output='screen',
            parameters=[config_power],
        ),
        Node(
            package='canbus_modules',
            executable='lamps.py',
            name='lamps',
            output='screen',
            parameters=[config_lamps],
        ),
        # ros2_socketcan replaces socketcan_bridge
        # Topics: /to_can_bus (send) and /from_can_bus (receive)
        Node(
            package='ros2_socketcan',
            executable='socket_can_sender_node',
            name='socket_can_sender',
            output='screen',
            parameters=[{'interface': 'can0'}],
            remappings=[('to_can_bus', '/to_can_bus')],
        ),
        Node(
            package='ros2_socketcan',
            executable='socket_can_receiver_node',
            name='socket_can_receiver',
            output='screen',
            parameters=[{'interface': 'can0'}],
            remappings=[('from_can_bus', '/from_can_bus')],
        ),
    ])