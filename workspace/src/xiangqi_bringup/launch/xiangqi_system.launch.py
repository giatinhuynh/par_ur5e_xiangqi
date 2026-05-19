"""
xiangqi_system.launch.py: Main launch file for the full Xiangqi robot system.

Starts all nodes with their config files loaded.
The hardware drivers (arm_drivers, moveit_config_driver) are expected to
already be running via the VXLab aliases before this launch file is invoked.

Usage:
  ros2 launch xiangqi_bringup xiangqi_system.launch.py
  ros2 launch xiangqi_bringup xiangqi_system.launch.py simulation_mode:=true
  ros2 launch xiangqi_bringup xiangqi_sim.launch.py self_play:=true engine_type:=fairystockfish
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
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
        default_value='20',
        description='Engine difficulty 1-20 (20 = max strength)',
    )
    self_play_arg = DeclareLaunchArgument(
        'self_play',
        default_value='false',
        description='Both sides played by AI (bot vs bot)',
    )
    robot_plays_red_arg = DeclareLaunchArgument(
        'robot_plays_red',
        default_value='true',
        description='Robot/AI plays Red side',
    )
    vision_config_arg = DeclareLaunchArgument(
        'vision_config_file',
        default_value='vision_config.yaml',
        description='Vision params YAML under xiangqi_bringup/config (e.g. vision_config_sim.yaml)',
    )
    move_to_initial_on_start_arg = DeclareLaunchArgument(
        'move_to_initial_pose_on_startup',
        default_value='true',
        description='After launch, move arm to initial pose once (hardware only)',
    )

    sim = LaunchConfiguration('simulation_mode')
    engine = LaunchConfiguration('engine_type')
    difficulty = LaunchConfiguration('difficulty')
    self_play = LaunchConfiguration('self_play')
    robot_plays_red = LaunchConfiguration('robot_plays_red')
    vision_config_file = LaunchConfiguration('vision_config_file')
    move_to_initial_on_start = LaunchConfiguration('move_to_initial_pose_on_startup')

    vision_cfg = PathJoinSubstitution([
        FindPackageShare('xiangqi_bringup'),
        'config',
        vision_config_file,
    ])

    return LaunchDescription([
        sim_arg, engine_arg, difficulty_arg, self_play_arg, robot_plays_red_arg,
        vision_config_arg, move_to_initial_on_start_arg,

        LogInfo(msg='Starting Xiangqi Robot System...'),

        # --- Reactive Layer (Tier 1) ---
        Node(
            package='xiangqi_vision',
            executable='vision_node',
            name='vision_node',
            parameters=[vision_cfg],
            output='screen',
        ),
        Node(
            package='xiangqi_manipulation',
            executable='manipulation_node',
            name='manipulation_node',
            parameters=[
                get_config('manipulation_config.yaml'),
                {
                    'simulation_mode': sim,
                    'move_to_initial_pose_on_startup': move_to_initial_on_start,
                },
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
                {'robot_plays_red': robot_plays_red},
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
                {
                    'engine_type': engine,
                    'self_play': self_play,
                    'robot_plays_red': robot_plays_red,
                    'simulation_mode': sim,
                },
            ],
            output='screen',
        ),

        # --- Cross-cutting: Dashboard ---
        Node(
            package='xiangqi_dashboard',
            executable='dashboard_node',
            name='dashboard_node',
            parameters=[{'port': 5000, 'simulation_mode': sim}],
            output='screen',
        ),
    ])
