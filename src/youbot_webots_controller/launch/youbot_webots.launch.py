import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    package_name = 'youbot_webots_controller'
    package_share = get_package_share_directory(package_name)
    
    # Путь к миру в нашем пакете
    webots_world = os.path.join(package_share, 'worlds', 'youbot.wbt')
    
    # Запуск Webots
    webots_process = ExecuteProcess(
        cmd=['webots', '--mode=pause', webots_world],
        output='screen'
    )
    
    # Узел контроллера youBot (запускаем с задержкой)
    youbot_controller_node = Node(
        package=package_name,
        executable='youbot_ros2_controller',
        name='youbot_ros2_controller',
        output='screen',
    )
    
    # Запуск контроллера через 5 секунд после старта Webots
    delayed_controller = TimerAction(
        period=5.0,
        actions=[youbot_controller_node]
    )
    
    # Узел для управления с клавиатуры
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        prefix='xterm -e',
        output='screen'
    )
    
    return LaunchDescription([
        webots_process,
        delayed_controller,
        teleop_node,
    ])