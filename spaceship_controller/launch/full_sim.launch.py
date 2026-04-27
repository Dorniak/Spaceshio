"""
Launch completo: simulador (profesor) + controlador de referencia.

ESTRATEGIAS DISPONIBLES
───────────────────────
  bangbang  Máquina de estados con corrección proporcional de heading
  pid       PID de heading + frenado predictivo  (RECOMENDADO)
  fuzzy     Lógica difusa Mamdani

USO
───
  # PID con target (por defecto):
  ros2 launch spaceship_controller full_sim.launch.py \
      target_x:=10.0 target_y:=8.0

  # Bang-bang sin viento (para comparar):
  ros2 launch spaceship_controller full_sim.launch.py \
      target_x:=10.0 target_y:=8.0 strategy:=bangbang wind_strength:=0.0

  # Fuzzy con viento oficial:
  ros2 launch spaceship_controller full_sim.launch.py \
      target_x:=10.0 target_y:=8.0 strategy:=fuzzy \
      wind_seed:=42 wind_strength:=1.2 wind_frequency:=0.15

  # Click en RViz (sin target inicial):
  ros2 launch spaceship_controller full_sim.launch.py strategy:=pid
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

_NO_TARGET = '-999.0'


def generate_launch_description():
    args = [
        DeclareLaunchArgument('target_x',       default_value=_NO_TARGET),
        DeclareLaunchArgument('target_y',       default_value=_NO_TARGET),
        DeclareLaunchArgument('strategy',       default_value='pid',
            description='bangbang | pid | fuzzy'),
        DeclareLaunchArgument('wind_seed',      default_value='0'),
        DeclareLaunchArgument('wind_strength',  default_value='0.0'),
        DeclareLaunchArgument('wind_frequency', default_value='0.15'),
    ]

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('spaceship_sim'),
                'launch', 'spaceship_sim.launch.py'
            )
        ),
        launch_arguments={
            'target_x':       LaunchConfiguration('target_x'),
            'target_y':       LaunchConfiguration('target_y'),
            'wind_seed':      LaunchConfiguration('wind_seed'),
            'wind_strength':  LaunchConfiguration('wind_strength'),
            'wind_frequency': LaunchConfiguration('wind_frequency'),
        }.items()
    )

    controller = Node(
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

    return LaunchDescription(args + [sim_launch, controller])
