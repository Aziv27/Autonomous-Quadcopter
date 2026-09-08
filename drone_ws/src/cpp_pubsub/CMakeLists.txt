# --- 1. THE BASICS ---
cmake_minimum_required(VERSION 3.8)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
project(cpp_pubsub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- 2. FIND PACKAGES (Dependencies) ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(ros2_control REQUIRED)
find_package(gazebo_ros2_control REQUIRED)

# --- 3. INSTALL PYTHON EXECUTABLES ---
install(PROGRAMS
  scripts/camera_processing.py
  scripts/check_imu.py
  scripts/drone_fsm.py
  scripts/imu_filter.py
  scripts/pid_mixer.py
  scripts/pid_tuner.py
  DESTINATION lib/${PROJECT_NAME}
)

# --- 4. INSTALL DIRECTORIES ---
install(DIRECTORY
  launch
  urdf
  rviz
  worlds
  meshes
  DESTINATION share/${PROJECT_NAME}
)

# --- 5. REQUIRED LAST LINE ---
ament_package()