# ============================================================================
# youbot_complete.launch.py - Полный запуск всей системы
# ============================================================================

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import os


def generate_launch_description():
    # Аргументы запуска
    world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value=PathJoinSubstitution([
            LaunchConfiguration('package_share', default='youbot_webots_controller'),
            'worlds',
            'youbot.wbt'
        ]),
        description='Path to Webots world file'
    )
    
    webots_controller_arg = DeclareLaunchArgument(
        'webots_controller',
        default_value='youbot_speed_controller',
        description='Name of Webots controller'
    )

    # Webots процесс
    webots_process = ExecuteProcess(
        cmd=['webots', '--stdout', '--stderr', LaunchConfiguration('world_file')],
        name='webots',
        output='screen'
    )
    
    # Даем время Webots запуститься перед запуском контроллера
    from launch.actions import TimerAction
    
    # Webots ROS2 контроллер (запускаем с задержкой)
    webots_controller_process = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'youbot_webots_controller', LaunchConfiguration('webots_controller')],
                name='webots_controller',
                output='screen'
            )
        ]
    )

    # Контроллер мобильной базы (также с задержкой)
    mobile_base_controller = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='youbot_webots_controller',
                executable='mobile_base_controller',
                name='mobile_base_controller',
                output='screen',
                parameters=[
                    {'linear_speed': 0.3},
                    {'angular_speed': 0.5},
                    {'safe_distance': 0.5}
                ]
            )
        ]
    )

    # Контроллер манипулятора
    arm_controller = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='youbot_webots_controller',
                executable='youbot_arm_controller',
                name='youbot_arm_controller',
                output='screen'
            )
        ]
    )

    # Commander (опционально)
    commander = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='youbot_webots_controller',
                executable='youbot_commander',
                name='youbot_commander',
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        world_file_arg,
        webots_controller_arg,
        webots_process,
        webots_controller_process,
        mobile_base_controller,
        arm_controller,
        commander
    ])