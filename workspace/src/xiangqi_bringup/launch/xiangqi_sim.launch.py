"""
xiangqi_sim.launch.py: Launch in full simulation mode (no real hardware).

Useful for testing game logic, AI, dashboard, and BT structure
without needing the physical robot and camera.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bringup_dir = get_package_share_directory('xiangqi_bringup')
    system_launch = os.path.join(bringup_dir, 'launch', 'xiangqi_system.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument(
            'engine_type', default_value='minimax',
            description='AI engine: fairystockfish or minimax (minimax needs no FSF binary)'),
        DeclareLaunchArgument(
            'self_play', default_value='false',
            description='Both sides played by AI (true = bot vs bot)'),
        DeclareLaunchArgument(
            'robot_plays_red', default_value='false',
            description='Robot/AI plays Red (false = human Red on dashboard in AI vs Human)'),
        DeclareLaunchArgument(
            'difficulty', default_value='20',
            description='Fairy-Stockfish skill level 1-20 (20 = max strength)'),

        LogInfo(msg='Starting Xiangqi in SIMULATION mode -- no hardware required'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(system_launch),
            launch_arguments={
                'simulation_mode': 'true',
                'vision_config_file': 'vision_config_sim.yaml',
                'engine_type': LaunchConfiguration('engine_type'),
                'self_play': LaunchConfiguration('self_play'),
                'robot_plays_red': LaunchConfiguration('robot_plays_red'),
                'difficulty': LaunchConfiguration('difficulty'),
            }.items(),
        ),
    ])
