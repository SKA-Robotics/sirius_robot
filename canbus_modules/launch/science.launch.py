from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    science_roboszpons_config = PathJoinSubstitution([
        FindPackageShare('canbus_modules'),
        'config', 'science_roboszpons.yaml'
    ])

    return LaunchDescription([
        Node(
            package='sirius_roboszpon_driver',
            executable='driver.py',
            name='science_roboszpon_driver',
            output='screen',
            parameters=[science_roboszpons_config],
            remappings=[
                ('/set_joint_states', '/science/command'),
                ('/joint_states', '/science/state'),
            ],
        ),
    ])