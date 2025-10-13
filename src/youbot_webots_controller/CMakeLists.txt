cmake_minimum_required(VERSION 3.8)
project(youbot_webots_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  youbot_webots_controller/webots_ros_bridge/webots_ros2_bridge_node.py
  youbot_webots_controller/platform_velocity_controller/platform_velocity_controller_node.py
  youbot_webots_controller/youbot_arm_controller/youbot_arm_controller_node.py
  youbot_webots_controller/gripper_controller/gripper_controller_node.py
  youbot_webots_controller/gui_controller/gui_controller_node.py
  youbot_webots_controller/youbot_odometry/youbot_odometry_node.py
  youbot_webots_controller/object_detector/image_processor_node.py
  youbot_webots_controller/triangulator/triangulator_node.py
  youbot_webots_controller/searching_for_grasping_controller/searching_SM.py
  youbot_webots_controller/coordinate_transform/coordinate_transform_service.py
  youbot_webots_controller/image_saver.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install world files
install(DIRECTORY
  worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

# Install UI files
install(DIRECTORY
  ui/
  DESTINATION share/${PROJECT_NAME}/ui
)

# Install hook files
install(FILES
  hook/ament_prefix_path.dsv
  DESTINATION share/${PROJECT_NAME}/hook
)

# Install resource files
install(FILES
  resource/${PROJECT_NAME}
  DESTINATION share/ament_index/resource_index/packages
)

# Install package.xml
install(FILES
  package.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()