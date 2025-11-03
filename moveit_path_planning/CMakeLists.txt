cmake_minimum_required(VERSION 3.8)
project(moveit_path_planning)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)

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
        rclcpp
)


install(TARGETS test_movements
        DESTINATION lib/${PROJECT_NAME})

install(TARGETS path_planning
        DESTINATION lib/${PROJECT_NAME})


if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_package()
