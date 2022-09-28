cmake_minimum_required(VERSION 3.8)
project(stonefish_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(Stonefish REQUIRED stonefish>=1.3.0)

# define libraries
add_library(stonefish_ros2 SHARED
#src/ROSInterface.cpp
#src/ROSControlInterface.cpp
src/ROSSimulationManager.cpp
src/ROSSimulationApp.cpp
#src/ROSScenarioParser.cpp
)
target_link_libraries(stonefish_ros2 ${Stonefish_LIBRARIES})
ament_target_dependencies(stonefish_ros2 rclcpp ${${PROJECT_NAME}_EXPORTED_TARGETS})
ament_export_libraries(stonefish_ros2)

# define exacutables
add_executable(parsed_simulator src/parsed_simulator.cpp)
ament_target_dependencies(parsed_simulator stonefish_ros2::stonefish_ros2)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
