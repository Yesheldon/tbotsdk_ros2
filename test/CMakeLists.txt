# 测试配置
enable_testing()

# 查找Google Test
find_package(GTest REQUIRED)
find_package(gmock REQUIRED)

# 包含头文件目录
include_directories(${CMAKE_SOURCE_DIR}/include)

# 静态功能测试
add_executable(test_connection static/test_connection.cpp)
target_link_libraries(test_connection 
    ${GTEST_LIBRARIES} 
    ${GTEST_MAIN_LIBRARIES}
    gmock
    pthread
)

add_executable(test_data_stream static/test_data_stream.cpp)
target_link_libraries(test_data_stream 
    ${GTEST_LIBRARIES} 
    ${GTEST_MAIN_LIBRARIES}
    gmock
    pthread
)

add_executable(test_robot_status static/test_robot_status.cpp)
target_link_libraries(test_robot_status 
    ${GTEST_LIBRARIES} 
    ${GTEST_MAIN_LIBRARIES}
    gmock
    pthread
)

add_executable(test_map_status static/test_map_status.cpp)
target_link_libraries(test_map_status 
    ${GTEST_LIBRARIES} 
    ${GTEST_MAIN_LIBRARIES}
    gmock
    pthread
)

# 动态功能测试
add_executable(test_navigation dynamic/test_navigation.cpp)
target_link_libraries(test_navigation 
    ${GTEST_LIBRARIES} 
    ${GTEST_MAIN_LIBRARIES}
    gmock
    pthread
)

add_executable(test_velocity_control dynamic/test_velocity_control.cpp)
target_link_libraries(test_velocity_control 
    ${GTEST_LIBRARIES} 
    ${GTEST_MAIN_LIBRARIES}
    gmock
    pthread
)

add_executable(test_waypoints dynamic/test_waypoints.cpp)
target_link_libraries(test_waypoints 
    ${GTEST_LIBRARIES} 
    ${GTEST_MAIN_LIBRARIES}
    gmock
    pthread
)

# 添加测试
add_test(NAME ConnectionTest COMMAND test_connection)
add_test(NAME DataStreamTest COMMAND test_data_stream)
add_test(NAME RobotStatusTest COMMAND test_robot_status)
add_test(NAME MapStatusTest COMMAND test_map_status)
add_test(NAME NavigationTest COMMAND test_navigation)
add_test(NAME VelocityControlTest COMMAND test_velocity_control)
add_test(NAME WaypointsTest COMMAND test_waypoints)

# 设置测试属性
set_tests_properties(ConnectionTest PROPERTIES TIMEOUT 30)
set_tests_properties(DataStreamTest PROPERTIES TIMEOUT 30)
set_tests_properties(RobotStatusTest PROPERTIES TIMEOUT 30)
set_tests_properties(MapStatusTest PROPERTIES TIMEOUT 30)
set_tests_properties(NavigationTest PROPERTIES TIMEOUT 30)
set_tests_properties(VelocityControlTest PROPERTIES TIMEOUT 30)
set_tests_properties(WaypointsTest PROPERTIES TIMEOUT 30) 