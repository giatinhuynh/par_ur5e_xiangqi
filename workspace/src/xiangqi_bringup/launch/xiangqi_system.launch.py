"""
xiangqi_system.launch.py: Main launch file for the full Xiangqi robot system.

Starts all nodes with their config files loaded.
The hardware drivers (arm_drivers, moveit_config_driver) are expected to
already be running via the VXLab aliases before this launch file is invoked.

Usage:
  ros2 launch xiangqi_bringup xiangqi_system.launch.py
  ros2 launch xiangqi_bringup xiangqi_system.launch.py simulation_mode:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

BRINGUP_DIR = get_package_share_directory('xiangqi_bringup')


def get_config(filename):
    return os.path.join(BRINGUP_DIR, 'config', filename)


def generate_launch_description():
    sim_arg = DeclareLaunchArgument(
        'simulation_mode',
        default_value='false',
        description='Run in simulation without real hardware',
    )
    engine_arg = DeclareLaunchArgument(
        'engine_type',
        default_value='fairystockfish',
        description='AI engine: fairystockfish or minimax',
    )
    difficulty_arg = DeclareLaunchArgument(
        'difficulty',
        default_value='15',
        description='Engine difficulty 1-20',
    )

    sim = LaunchConfiguration('simulation_mode')
    engine = LaunchConfiguration('engine_type')
    difficulty = LaunchConfiguration('difficulty')

    return LaunchDescription([
        sim_arg, engine_arg, difficulty_arg,

        LogInfo(msg='Starting Xiangqi Robot System...'),

        # --- Reactive Layer (Tier 1) ---
        Node(
            package='xiangqi_vision',
            executable='vision_node',
            name='vision_node',
            parameters=[get_config('vision_config.yaml')],
            output='screen',
        ),
        Node(
            package='xiangqi_manipulation',
            executable='manipulation_node',
            name='manipulation_node',
            parameters=[
                get_config('manipulation_config.yaml'),
                {'simulation_mode': sim},
            ],
            output='screen',
        ),
        Node(
            package='xiangqi_manipulation',
            executable='gripper_controller_node',
            name='gripper_controller_node',
            parameters=[
                get_config('manipulation_config.yaml'),
                {'simulation_mode': sim},
            ],
            output='screen',
        ),
        Node(
            package='xiangqi_manipulation',
            executable='safety_monitor_node',
            name='safety_monitor_node',
            output='screen',
        ),

        # --- Sequencing Layer (Tier 2) ---
        Node(
            package='xiangqi_planner',
            executable='task_planner_node',
            name='task_planner_node',
            parameters=[
                get_config('planner_config.yaml'),
                get_config('robot_side.yaml'),
            ],
            output='screen',
        ),

        # --- Deliberative Layer (Tier 3) ---
        Node(
            package='xiangqi_ai',
            executable='ai_engine_node',
            name='ai_engine_node',
            parameters=[
                get_config('game_config.yaml'),
                {'engine_type': engine, 'difficulty': difficulty},
            ],
            output='screen',
        ),
        Node(
            package='xiangqi_ai',
            executable='game_manager_node',
            name='game_manager_node',
            parameters=[
                get_config('game_config.yaml'),
                get_config('robot_side.yaml'),
                {'engine_type': engine},
            ],
            output='screen',
        ),

        # --- Cross-cutting: Dashboard ---
        Node(
            package='xiangqi_dashboard',
            executable='dashboard_node',
            name='dashboard_node',
            parameters=[{'port': 5000}],
            output='screen',
        ),
    ])
