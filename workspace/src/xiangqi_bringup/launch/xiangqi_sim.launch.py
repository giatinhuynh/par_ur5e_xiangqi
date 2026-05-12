"""
xiangqi_sim.launch.py: Launch in full simulation mode (no real hardware).

Useful for testing game logic, AI, dashboard, and BT structure
without needing the physical robot and camera.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bringup_dir = get_package_share_directory('xiangqi_bringup')
    system_launch = os.path.join(bringup_dir, 'launch', 'xiangqi_system.launch.py')

    return LaunchDescription([
        LogInfo(msg='Starting Xiangqi in SIMULATION mode -- no hardware required'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(system_launch),
            launch_arguments={
                'simulation_mode': 'true',
                'engine_type': 'minimax',   # Use minimax by default in sim (no Fairy-Stockfish needed)
                'difficulty': '4',
            }.items(),
        ),
    ])
