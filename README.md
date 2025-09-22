# 1. Создайте рабочее пространство и клонируйте репозиторий
mkdir -p ~/youbot_ws/src
cd ~/youbot_ws/src
git clone https://github.com/yourusername/youbot_ros2_controller.git

# 2. Установите переменные окружения
echo 'export WEBOTS_HOME=/usr/local/webots' >> ~/.bashrc
source ~/.bashrc

# 3. Установите зависимости
cd ~/youbot_ws
sudo apt install ros-humble-teleop-twist-keyboard
rosdep install -i --from-path src --rosdistro humble -y

# 4. Соберите пакет
colcon build --packages-select youbot_webots_controller
source install/setup.bash

# 5. Запустите
ros2 launch youbot_webots_controller youbot_manual.launch.py
