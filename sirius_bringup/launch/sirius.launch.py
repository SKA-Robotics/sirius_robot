from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_state_frequency_arg = DeclareLaunchArgument(
        'robot_state_frequency', default_value='100')

    sirius_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('sirius_description'),
            'launch', 'sirius_description.launch.py',
        ])),
    )

    hardware_interfaces = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('sirius_bringup'),
            'launch', 'hardware_interfaces.launch.py',
        ])),
    )

    telemetry = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('rxsio_telemetry'),
            'launch', 'telemetry.launch.py',
        ])),
    )

    gnss = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('sirius_navigation'),
            'launch', 'gnss.launch.py',
        ])),
    )

    return LaunchDescription([
        robot_state_frequency_arg,
        sirius_description,
        hardware_interfaces,
        telemetry,
        gnss,
    ])
