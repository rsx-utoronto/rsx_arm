cmake_minimum_required(VERSION 3.8)
project(moveit_path_planning)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(arm_msgs REQUIRED)

# Include directories
include_directories(include)

add_executable(test_movements src/test_movements.cpp)

add_executable(path_planning
        include/PathPlannerNode.hpp

        src/PathPlannerNode.cpp
        src/publishPath.cpp
        include/PathPlannerNode.hpp
)

target_include_directories(test_movements PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(test_movements PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
        test_movements
        moveit_ros_planning_interface
        rclcpp
)
target_compile_features(path_planning PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
        path_planning
        moveit_ros_planning_interface
        arm_msgs
        rclcpp
)


install(TARGETS test_movements
        DESTINATION lib/${PROJECT_NAME})

install(TARGETS path_planning
        DESTINATION lib/${PROJECT_NAME})



# Planner server library (for component)
add_library(planner_server_component SHARED
  src/planner_server.cpp
)

target_include_directories(planner_server_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(planner_server_component PUBLIC cxx_std_17)

ament_target_dependencies(planner_server_component
  rclcpp
  rclcpp_action
  rclcpp_components
  moveit_ros_planning_interface
  moveit_core
  moveit_msgs
  moveit_ros_planning
  control_msgs
  trajectory_msgs
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2_ros
  tf2_geometry_msgs
  arm_msgs
)

rclcpp_components_register_node(planner_server_component
  PLUGIN "moveit_path_planning::PlannerServer"
  EXECUTABLE planner_server
)

# Planner server standalone executable
add_executable(planner_server_node src/planner_server_main.cpp)

target_compile_features(planner_server_node PUBLIC cxx_std_17)

target_link_libraries(planner_server_node planner_server_component)

ament_target_dependencies(planner_server_node
  rclcpp
)

# Install targets
install(TARGETS
  planner_server_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  planner_server_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install include directory
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  # Linters
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Unit tests
  ament_add_gtest(test_planner_server
    test/test_planner_server.cpp
  )
  
  if(TARGET test_planner_server)
    target_link_libraries(test_planner_server
      planner_server_component
    )
    ament_target_dependencies(test_planner_server
      rclcpp
      moveit_ros_planning_interface
      arm_msgs
    )
  endif()
endif()

ament_package()
