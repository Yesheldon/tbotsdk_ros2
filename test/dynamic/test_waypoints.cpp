#include <gtest/gtest.h>
#include "tbot_sdk/TBotSDK.h"
#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <thread>

using namespace TBot;

class RealWaypointsTest : public ::testing::Test {
protected:
    void SetUp() override {
        tbot_ = std::make_unique<TBotSDK>("192.168.8.110");
        tbot_->connect();
    }
    
    void TearDown() override {
        if (tbot_->isConnected()) {
            tbot_->disconnect();
        }
        tbot_.reset();
    }
    
    std::unique_ptr<TBotSDK> tbot_;
};

// 测试保存路径点
TEST_F(RealWaypointsTest, TestSaveWaypoint) {
    // 保存一个测试路径点
    bool result = tbot_->saveWaypoint("test_waypoint", {1.0f, 2.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, "Test waypoint description");
    EXPECT_TRUE(result);
    
    std::cout << "Waypoint 'test_waypoint' saved" << std::endl;
}

// 测试保存当前位置为路径点
TEST_F(RealWaypointsTest, TestSaveCurrentPositionAsWaypoint) {
    // 保存当前位置为路径点
    bool result = tbot_->saveCurrentPositionAsWaypoint("current_position", "Current robot position");
    EXPECT_TRUE(result);
    
    std::cout << "Current position saved as waypoint" << std::endl;
}

// 测试加载路径点
TEST_F(RealWaypointsTest, TestLoadWaypoint) {
    bool callback_called = false;
    
    bool result = tbot_->loadWaypoint("test_waypoint", [&callback_called](const Waypoint& waypoint) {
        callback_called = true;
        
        // 验证路径点信息
        EXPECT_EQ(waypoint.name, "test_waypoint");
        EXPECT_FINITE(waypoint.pose.position.x);
        EXPECT_FINITE(waypoint.pose.position.y);
        EXPECT_FINITE(waypoint.pose.position.z);
        EXPECT_FINITE(waypoint.pose.orientation.x);
        EXPECT_FINITE(waypoint.pose.orientation.y);
        EXPECT_FINITE(waypoint.pose.orientation.z);
        EXPECT_FINITE(waypoint.pose.orientation.w);
        
        std::cout << "Loaded waypoint: " << waypoint.name << std::endl;
        std::cout << "Position: (" << waypoint.pose.position.x << ", " 
                  << waypoint.pose.position.y << ", " << waypoint.pose.position.z << ")" << std::endl;
    });
    
    EXPECT_TRUE(result);
    EXPECT_TRUE(callback_called);
}

// 测试获取所有路径点
TEST_F(RealWaypointsTest, TestGetAllWaypoints) {
    auto waypoints = tbot_->getAllWaypoints();
    
    // 验证路径点列表
    EXPECT_GE(waypoints.size(), 0);
    
    std::cout << "Found " << waypoints.size() << " waypoints:" << std::endl;
    for (const auto& waypoint : waypoints) {
        EXPECT_FALSE(waypoint.name.empty());
        EXPECT_FINITE(waypoint.pose.position.x);
        EXPECT_FINITE(waypoint.pose.position.y);
        EXPECT_FINITE(waypoint.pose.position.z);
        
        std::cout << "  - " << waypoint.name << " at (" 
                  << waypoint.pose.position.x << ", " 
                  << waypoint.pose.position.y << ", " 
                  << waypoint.pose.position.z << ")" << std::endl;
    }
}

// 测试导航到路径点
TEST_F(RealWaypointsTest, TestNavigateToWaypoint) {
    // 先获取所有路径点
    auto waypoints = tbot_->getAllWaypoints();
    
    if (!waypoints.empty()) {
        // 导航到第一个路径点
        bool result = tbot_->navigateToWaypoint(waypoints[0].name);
        EXPECT_TRUE(result);
        
        std::cout << "Navigating to waypoint: " << waypoints[0].name << std::endl;
        
        // 等待导航开始
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        if (tbot_->isNavigating()) {
            // 等待一段时间让导航进行
            std::this_thread::sleep_for(std::chrono::seconds(5));
            
            // 停止导航
            tbot_->stopNavigation();
        }
    }
}

// 测试同步导航到路径点
TEST_F(RealWaypointsTest, TestNavigateToWaypointAndWait) {
    auto waypoints = tbot_->getAllWaypoints();
    
    if (!waypoints.empty()) {
        // 同步导航到路径点，设置较短的超时时间
        bool result = tbot_->navigateToWaypointAndWait(waypoints[0].name, 10);
        EXPECT_TRUE(result);
        
        std::cout << "Navigation to waypoint completed or timed out" << std::endl;
    }
}

// 测试删除路径点
TEST_F(RealWaypointsTest, TestDeleteWaypoint) {
    // 先保存一个临时路径点
    tbot_->saveWaypoint("temp_waypoint", {0.5f, 0.5f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, "Temporary waypoint");
    
    // 删除这个路径点
    bool result = tbot_->deleteWaypoint("temp_waypoint");
    EXPECT_TRUE(result);
    
    std::cout << "Temporary waypoint deleted" << std::endl;
}

// 测试路径点内容验证
TEST_F(RealWaypointsTest, TestWaypointContent) {
    // 保存一个测试路径点
    tbot_->saveWaypoint("content_test", {3.0f, 4.0f, 0.0f}, {0.0f, 0.0f, 0.707f, 0.707f}, "Content test waypoint");
    
    // 加载并验证路径点内容
    tbot_->loadWaypoint("content_test", [](const Waypoint& waypoint) {
        EXPECT_EQ(waypoint.name, "content_test");
        EXPECT_NEAR(waypoint.pose.position.x, 3.0f, 0.1f);
        EXPECT_NEAR(waypoint.pose.position.y, 4.0f, 0.1f);
        EXPECT_NEAR(waypoint.pose.position.z, 0.0f, 0.1f);
        EXPECT_NEAR(waypoint.pose.orientation.x, 0.0f, 0.1f);
        EXPECT_NEAR(waypoint.pose.orientation.y, 0.0f, 0.1f);
        EXPECT_NEAR(waypoint.pose.orientation.z, 0.707f, 0.1f);
        EXPECT_NEAR(waypoint.pose.orientation.w, 0.707f, 0.1f);
        EXPECT_EQ(waypoint.description, "Content test waypoint");
        
        // 验证四元数归一化
        float quaternion_norm = waypoint.pose.orientation.x * waypoint.pose.orientation.x +
                               waypoint.pose.orientation.y * waypoint.pose.orientation.y +
                               waypoint.pose.orientation.z * waypoint.pose.orientation.z +
                               waypoint.pose.orientation.w * waypoint.pose.orientation.w;
        EXPECT_NEAR(quaternion_norm, 1.0f, 0.1f);
    });
    
    // 清理测试路径点
    tbot_->deleteWaypoint("content_test");
}

// 测试路径点参数验证
TEST_F(RealWaypointsTest, TestWaypointParameterValidation) {
    // 测试无效路径点名称
    bool result = tbot_->loadWaypoint("", [](const Waypoint& waypoint) {
        FAIL() << "Should not load empty waypoint name";
    });
    EXPECT_FALSE(result);
    
    // 测试删除不存在的路径点
    bool delete_result = tbot_->deleteWaypoint("non_existent_waypoint");
    EXPECT_FALSE(delete_result);
    
    // 测试导航到不存在的路径点
    bool nav_result = tbot_->navigateToWaypoint("non_existent_waypoint");
    EXPECT_FALSE(nav_result);
}

// 测试路径点边界条件
TEST_F(RealWaypointsTest, TestWaypointBoundaryConditions) {
    // 测试保存路径点时的边界值
    bool result = tbot_->saveWaypoint("boundary_test", {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, "");
    EXPECT_TRUE(result);
    
    // 测试保存路径点时的最大距离
    bool far_result = tbot_->saveWaypoint("far_waypoint", {100.0f, 100.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, "Far waypoint");
    EXPECT_TRUE(far_result);
    
    // 清理测试路径点
    tbot_->deleteWaypoint("boundary_test");
    tbot_->deleteWaypoint("far_waypoint");
}

// 测试路径点精度验证
TEST_F(RealWaypointsTest, TestWaypointPrecision) {
    // 保存一个精确的路径点
    tbot_->saveWaypoint("precision_test", {1.234f, 2.345f, 0.0f}, {0.1f, 0.2f, 0.3f, 0.9f}, "Precision test");
    
    // 验证保存的精度
    tbot_->loadWaypoint("precision_test", [](const Waypoint& waypoint) {
        EXPECT_NEAR(waypoint.pose.position.x, 1.234f, 0.01f);
        EXPECT_NEAR(waypoint.pose.position.y, 2.345f, 0.01f);
        EXPECT_NEAR(waypoint.pose.position.z, 0.0f, 0.01f);
        
        // 验证四元数归一化
        float quaternion_norm = waypoint.pose.orientation.x * waypoint.pose.orientation.x +
                               waypoint.pose.orientation.y * waypoint.pose.orientation.y +
                               waypoint.pose.orientation.z * waypoint.pose.orientation.z +
                               waypoint.pose.orientation.w * waypoint.pose.orientation.w;
        EXPECT_NEAR(quaternion_norm, 1.0f, 0.1f);
    });
    
    // 清理测试路径点
    tbot_->deleteWaypoint("precision_test");
}

// 测试路径点操作错误处理
TEST_F(RealWaypointsTest, TestWaypointErrorHandling) {
    // 测试在未连接状态下操作路径点
    auto disconnected_tbot = std::make_unique<TBotSDK>("192.168.8.110");
    
    bool save_result = disconnected_tbot->saveWaypoint("test", {1.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, "test");
    EXPECT_FALSE(save_result);
    
    bool load_result = disconnected_tbot->loadWaypoint("test", [](const Waypoint& waypoint) {
        FAIL() << "Should not load waypoint when not connected";
    });
    EXPECT_FALSE(load_result);
    
    bool nav_result = disconnected_tbot->navigateToWaypoint("test");
    EXPECT_FALSE(nav_result);
}

// 测试路径点管理
TEST_F(RealWaypointsTest, TestWaypointManagement) {
    // 创建多个测试路径点
    std::vector<std::string> test_waypoints = {"wp1", "wp2", "wp3"};
    
    for (const auto& name : test_waypoints) {
        tbot_->saveWaypoint(name, {1.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, "Test waypoint");
    }
    
    // 获取所有路径点
    auto waypoints = tbot_->getAllWaypoints();
    EXPECT_GE(waypoints.size(), test_waypoints.size());
    
    // 验证所有测试路径点都存在
    for (const auto& test_name : test_waypoints) {
        bool found = false;
        for (const auto& waypoint : waypoints) {
            if (waypoint.name == test_name) {
                found = true;
                break;
            }
        }
        EXPECT_TRUE(found) << "Test waypoint " << test_name << " not found";
    }
    
    // 清理测试路径点
    for (const auto& name : test_waypoints) {
        tbot_->deleteWaypoint(name);
    }
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
} 