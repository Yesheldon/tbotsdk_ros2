cmake_minimum_required(VERSION 3.8)
project(tbot_sdk)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)

# Find system dependencies
find_package(CURL REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)
find_package(ZLIB REQUIRED)

# Include directories
include_directories(include)
include_directories(${CURL_INCLUDE_DIRS})
include_directories(${JSONCPP_INCLUDE_DIRS})

# Create library
add_library(${PROJECT_NAME}
  src/TBotSDK.cpp
)

# Link libraries
target_link_libraries(${PROJECT_NAME}
  ${CURL_LIBRARIES}
  ${JSONCPP_LIBRARIES}
  ZLIB::ZLIB
)

# Link ROS2 libraries
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
  std_srvs
)

# Create executables
add_executable(test_api src/test_api.cpp)
target_link_libraries(test_api ${PROJECT_NAME})

add_executable(tbot_node src/tbot_node.cpp)
target_link_libraries(tbot_node ${PROJECT_NAME})

add_executable(test_data_stream src/test_data_stream.cpp)
target_link_libraries(test_data_stream ${PROJECT_NAME})

add_executable(test_timeout src/test_timeout.cpp)
target_link_libraries(test_timeout ${PROJECT_NAME})

add_executable(test_waypoints src/test_waypoints.cpp)
target_link_libraries(test_waypoints ${PROJECT_NAME})

add_executable(waypoint_service src/waypoint_service.cpp)
target_link_libraries(waypoint_service ${PROJECT_NAME})

# Install targets
install(TARGETS
  ${PROJECT_NAME}
  test_api
  tbot_node
  test_data_stream
  test_timeout
  test_waypoints
  waypoint_service
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# 启用测试编译
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  ament_lint_auto_find_test_dependencies()
  
  # 包含测试目录的CMakeLists.txt
  add_subdirectory(test)
  
  # 添加SDK测试
  ament_add_gtest(test_tbot_sdk test/test_tbot_sdk.cpp)
  target_link_libraries(test_tbot_sdk ${PROJECT_NAME})
  ament_target_dependencies(test_tbot_sdk
    rclcpp
    std_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
    tf2
    tf2_ros
  )
  
  # 添加节点测试
  ament_add_gtest(test_tbot_node test/test_tbot_node.cpp)
  target_link_libraries(test_tbot_node ${PROJECT_NAME})
  ament_target_dependencies(test_tbot_node
    rclcpp
    std_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
    tf2
    tf2_ros
  )
endif()

ament_package() 