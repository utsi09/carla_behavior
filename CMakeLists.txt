cmake_minimum_required(VERSION 3.8)
project(carla_behavior)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 빌드시 compile_commands.json 생성
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)


# 실행 파일 빌드
add_executable(carla_behavior_node
  src/main.cpp
  src/checkRiskLevel.cpp

)

# include 디렉토리 추가
target_include_directories(carla_behavior_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 의존성 링크
ament_target_dependencies(carla_behavior_node
  rclcpp
  behaviortree_cpp
  behaviortree_ros2
  sensor_msgs
  ament_index_cpp
) 


# 실행 파일 설치
install(
  TARGETS carla_behavior_node
  DESTINATION lib/${PROJECT_NAME}
)

# behavior_trees 디렉토리 설치
install(
  DIRECTORY behavior_trees/
  DESTINATION share/${PROJECT_NAME}/behavior_trees
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_cppcheck_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
