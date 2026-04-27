"""Launch solo del controlador (spaceship_sim ya debe estar corriendo)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_NO_TARGET = '-999.0'

def generate_launch_description():
    args = [
        DeclareLaunchArgument('target_x',  default_value=_NO_TARGET),
        DeclareLaunchArgument('target_y',  default_value=_NO_TARGET),
        DeclareLaunchArgument('strategy',  default_value='pid',
            description='bangbang | pid | fuzzy'),
    ]
    return LaunchDescription(args + [
        Node(
            package='spaceship_controller',
            executable='controller',
            name='spaceship_controller',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'target_x': LaunchConfiguration('target_x'),
                'target_y': LaunchConfiguration('target_y'),
                'strategy': LaunchConfiguration('strategy'),
            }]
        )
    ])
