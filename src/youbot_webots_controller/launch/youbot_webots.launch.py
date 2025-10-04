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
            LaunchConfiguration('package_share', default='src/youbot_webots_controller'),
            'worlds',
            'youbot.wbt'
        ]),
        description='Path to Webots world file'
    )
    
    webots_controller_arg = DeclareLaunchArgument(
        'webots_controller',
        default_value='webots_ros2_bridge',
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

    # Контроллер мобильной платформы (также с задержкой)
    mobile_base_controller = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='youbot_webots_controller',
                executable='platform_velocity_controller',
                name='platform_velocity_controller',
                output='screen',
                parameters=[
                    {'linear_speed': 0.3},
                    {'angular_speed': 0.5},
                    {'safe_distance': 0.5}
                ]
            )
        ]
    )

    # Одометрия
    youbot_odometry = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='youbot_webots_controller',
                executable='youbot_odometry',
                name='youbot_odometry',
                output='screen'
            )
        ]
    )

    # youbot_mapping = TimerAction(
    #     period=8.0,
    #     actions=[
    #         Node(
    #             package='youbot_webots_controller',
    #             executable='mapping_node',
    #             name='mapping_node',
    #             output='screen'
    #         )
    #     ]
    # )

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

    # Контроллер схвата
    gripper_controller = TimerAction(
        period=9.0,
        actions=[
            Node(
                package='youbot_webots_controller',
                executable='gripper_controller',
                name='gripper_controller',
                output='screen'
            )
        ]
    )

    image_processor = TimerAction(
        period=9.0,
        actions=[
            Node(
                package='youbot_webots_controller',
                executable='image_processor',
                name='image_processor',
                output='screen'
            )
        ]
    )

    triangulator_node = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='youbot_webots_controller',
                executable='triangulator_node',
                name='triangulator_node',
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
        youbot_odometry,
        arm_controller,
        gripper_controller,
        image_processor,
        triangulator_node
    ])