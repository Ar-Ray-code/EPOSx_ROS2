cmake_minimum_required(VERSION 3.5)
project(epos_single)

set(CMAKE_CXX_STANDARD 14)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(epos_node
  src/epos_single.cpp
  src/epos_base.cpp
)
include_directories(epos_node
  include/epos_single
)

target_link_libraries(epos_node -lEposCmd)

ament_target_dependencies(epos_node
rclcpp
std_msgs
)

install(TARGETS epos_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
