from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    hw_config = PathJoinSubstitution([
        FindPackageShare('sirius_bringup'),
        'config', 'hardware_interfaces.yaml',
    ])

    odrive_driver = Node(
        package='sirius_odrive_driver',
        executable='driver.py',
        name='odrive_driver',
        output='screen',
        parameters=[hw_config],
    )

    relaxing_middleware = Node(
        package='sirius_odrive_driver',
        executable='relaxing_middleware.py',
        name='relaxing_middleware',
        output='screen',
        parameters=[hw_config],
    )

    canbus_all = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('canbus_modules'),
            'launch', 'all.launch.py',
        ])),
    )

    return LaunchDescription([
        odrive_driver,
        relaxing_middleware,
        canbus_all,
    ])
