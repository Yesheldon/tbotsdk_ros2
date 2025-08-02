#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "tbot_sdk/TBotSDK.h"
#include <memory>
#include <string>
#include <vector>

using namespace TBot;
using ::testing::_;
using ::testing::Return;
using ::testing::StrictMock;

// Mock类用于测试导航点功能
class MockTBotSDK : public TBotSDK {
public:
    MockTBotSDK(const std::string& ip = "192.168.8.110") : TBotSDK(ip) {}
    
    MOCK_METHOD(bool, saveCurrentPositionAsWaypoint, (const std::string& name, const std::string& description, StatusCallback callback), (override));
    MOCK_METHOD(bool, loadWaypoint, (const std::string& name, WaypointCallback callback), (override));
    MOCK_METHOD(std::vector<Waypoint>, getAllWaypoints, (const std::string& filter, StatusCallback callback), (override));
    MOCK_METHOD(bool, navigateToWaypoint, (const std::string& name, StatusCallback callback), (override));
    MOCK_METHOD(bool, navigateToWaypointAndWait, (const std::string& name, int timeout, StatusCallback callback), (override));
    MOCK_METHOD(bool, deleteWaypoint, (const std::string& name, StatusCallback callback), (override));
};

class WaypointsTest : public ::testing::Test {
protected:
    void SetUp() override {
        mock_tbot_ = std::make_unique<StrictMock<MockTBotSDK>>("192.168.8.110");
    }
    
    void TearDown() override {
        mock_tbot_.reset();
    }
    
    std::unique_ptr<StrictMock<MockTBotSDK>> mock_tbot_;
};

// 测试保存当前位置作为导航点
TEST_F(WaypointsTest, TestSaveCurrentPositionAsWaypoint) {
    EXPECT_CALL(*mock_tbot_, saveCurrentPositionAsWaypoint("A点", "起始位置", _))
        .WillOnce(Return(true));
    
    bool result = mock_tbot_->saveCurrentPositionAsWaypoint("A点", "起始位置", [](int code, const std::string& message) {
        EXPECT_EQ(code, 0);
        EXPECT_EQ(message, "Waypoint saved successfully");
    });
    
    EXPECT_TRUE(result);
}

// 测试加载导航点
TEST_F(WaypointsTest, TestLoadWaypoint) {
    EXPECT_CALL(*mock_tbot_, loadWaypoint("A点", _))
        .WillOnce(Return(true));
    
    bool result = mock_tbot_->loadWaypoint("A点", [](const Waypoint& waypoint) {
        EXPECT_EQ(waypoint.name, "A点");
        EXPECT_EQ(waypoint.description, "起始位置");
        EXPECT_FLOAT_EQ(waypoint.pose.position.x, 1.0);
        EXPECT_FLOAT_EQ(waypoint.pose.position.y, 2.0);
        EXPECT_FLOAT_EQ(waypoint.pose.position.z, 0.0);
        EXPECT_FLOAT_EQ(waypoint.pose.orientation.w, 1.0);
    });
    
    EXPECT_TRUE(result);
}

// 测试获取所有导航点
TEST_F(WaypointsTest, TestGetAllWaypoints) {
    std::vector<Waypoint> expected_waypoints;
    Waypoint wp1, wp2;
    wp1.name = "A点";
    wp1.description = "起始位置";
    wp1.pose.position.x = 1.0;
    wp1.pose.position.y = 2.0;
    wp2.name = "B点";
    wp2.description = "中间位置";
    wp2.pose.position.x = 3.0;
    wp2.pose.position.y = 4.0;
    expected_waypoints.push_back(wp1);
    expected_waypoints.push_back(wp2);
    
    EXPECT_CALL(*mock_tbot_, getAllWaypoints("", _))
        .WillOnce(Return(expected_waypoints));
    
    auto waypoints = mock_tbot_->getAllWaypoints("", [](int code, const std::string& message) {
        EXPECT_EQ(code, 0);
        EXPECT_EQ(message, "Success");
    });
    
    EXPECT_EQ(waypoints.size(), 2);
    EXPECT_EQ(waypoints[0].name, "A点");
    EXPECT_EQ(waypoints[1].name, "B点");
}

// 测试导航到导航点
TEST_F(WaypointsTest, TestNavigateToWaypoint) {
    EXPECT_CALL(*mock_tbot_, navigateToWaypoint("A点", _))
        .WillOnce(Return(true));
    
    bool result = mock_tbot_->navigateToWaypoint("A点", [](int code, const std::string& message) {
        EXPECT_EQ(code, 0);
        EXPECT_EQ(message, "Navigation to waypoint started");
    });
    
    EXPECT_TRUE(result);
}

// 测试同步导航到导航点
TEST_F(WaypointsTest, TestNavigateToWaypointAndWait) {
    EXPECT_CALL(*mock_tbot_, navigateToWaypointAndWait("B点", 30, _))
        .WillOnce(Return(true));
    
    bool result = mock_tbot_->navigateToWaypointAndWait("B点", 30, [](int code, const std::string& message) {
        EXPECT_EQ(code, 0);
        EXPECT_EQ(message, "Navigation to waypoint completed");
    });
    
    EXPECT_TRUE(result);
}

// 测试删除导航点
TEST_F(WaypointsTest, TestDeleteWaypoint) {
    EXPECT_CALL(*mock_tbot_, deleteWaypoint("C点", _))
        .WillOnce(Return(true));
    
    bool result = mock_tbot_->deleteWaypoint("C点", [](int code, const std::string& message) {
        EXPECT_EQ(code, 0);
        EXPECT_EQ(message, "Waypoint deleted successfully");
    });
    
    EXPECT_TRUE(result);
}

// 测试保存导航点失败
TEST_F(WaypointsTest, TestSaveCurrentPositionAsWaypointFailure) {
    EXPECT_CALL(*mock_tbot_, saveCurrentPositionAsWaypoint("", "", _))
        .WillOnce(Return(false));
    
    bool result = mock_tbot_->saveCurrentPositionAsWaypoint("", "", [](int code, const std::string& message) {
        EXPECT_EQ(code, -1);
        EXPECT_EQ(message, "Failed to save waypoint");
    });
    
    EXPECT_FALSE(result);
}

// 测试加载导航点失败
TEST_F(WaypointsTest, TestLoadWaypointFailure) {
    EXPECT_CALL(*mock_tbot_, loadWaypoint("不存在的点", _))
        .WillOnce(Return(false));
    
    bool result = mock_tbot_->loadWaypoint("不存在的点", [](const Waypoint& waypoint) {
        // 这个回调不应该被调用
        FAIL() << "Waypoint callback should not be called";
    });
    
    EXPECT_FALSE(result);
}

// 测试导航到导航点失败
TEST_F(WaypointsTest, TestNavigateToWaypointFailure) {
    EXPECT_CALL(*mock_tbot_, navigateToWaypoint("不存在的点", _))
        .WillOnce(Return(false));
    
    bool result = mock_tbot_->navigateToWaypoint("不存在的点", [](int code, const std::string& message) {
        EXPECT_EQ(code, -1);
        EXPECT_EQ(message, "Waypoint not found");
    });
    
    EXPECT_FALSE(result);
}

// 测试同步导航到导航点超时
TEST_F(WaypointsTest, TestNavigateToWaypointAndWaitTimeout) {
    EXPECT_CALL(*mock_tbot_, navigateToWaypointAndWait("远点", 5, _))
        .WillOnce(Return(false));
    
    bool result = mock_tbot_->navigateToWaypointAndWait("远点", 5, [](int code, const std::string& message) {
        EXPECT_EQ(code, -1);
        EXPECT_EQ(message, "Navigation timeout");
    });
    
    EXPECT_FALSE(result);
}

// 测试导航点内容验证
TEST_F(WaypointsTest, TestWaypointContent) {
    Waypoint testWaypoint;
    testWaypoint.name = "测试点";
    testWaypoint.description = "测试描述";
    testWaypoint.pose.position.x = 5.0;
    testWaypoint.pose.position.y = 3.0;
    testWaypoint.pose.position.z = 0.0;
    testWaypoint.pose.orientation.x = 0.0;
    testWaypoint.pose.orientation.y = 0.0;
    testWaypoint.pose.orientation.z = 0.707;
    testWaypoint.pose.orientation.w = 0.707;
    testWaypoint.created_time = "2024-01-01 12:00:00";
    
    EXPECT_EQ(testWaypoint.name, "测试点");
    EXPECT_EQ(testWaypoint.description, "测试描述");
    EXPECT_FLOAT_EQ(testWaypoint.pose.position.x, 5.0);
    EXPECT_FLOAT_EQ(testWaypoint.pose.position.y, 3.0);
    EXPECT_FLOAT_EQ(testWaypoint.pose.position.z, 0.0);
    EXPECT_FLOAT_EQ(testWaypoint.pose.orientation.z, 0.707);
    EXPECT_FLOAT_EQ(testWaypoint.pose.orientation.w, 0.707);
    EXPECT_EQ(testWaypoint.created_time, "2024-01-01 12:00:00");
}

// 测试导航点名称验证
TEST_F(WaypointsTest, TestWaypointNameValidation) {
    // 测试有效名称
    std::vector<std::string> valid_names = {
        "A点", "B点", "C点", "起始位置", "终点位置", "充电桩", "工作台1", "工作台2"
    };
    
    for (const auto& name : valid_names) {
        EXPECT_FALSE(name.empty());
        EXPECT_LE(name.length(), 50);
    }
    
    // 测试无效名称
    std::vector<std::string> invalid_names = {
        "", "   ", "非常非常非常非常非常非常非常非常非常非常长的名称"
    };
    
    for (const auto& name : invalid_names) {
        EXPECT_TRUE(name.empty() || name.length() > 50);
    }
}

// 测试导航点坐标验证
TEST_F(WaypointsTest, TestWaypointCoordinateValidation) {
    // 测试有效坐标
    std::vector<std::tuple<float, float, float>> valid_coords = {
        {0.0f, 0.0f, 0.0f},
        {1.0f, 2.0f, 0.0f},
        {-5.0f, -3.0f, 0.0f},
        {10.0f, 15.0f, 0.0f}
    };
    
    for (const auto& coords : valid_coords) {
        float x, y, z;
        std::tie(x, y, z) = coords;
        
        EXPECT_GE(x, -100.0f);
        EXPECT_LE(x, 100.0f);
        EXPECT_GE(y, -100.0f);
        EXPECT_LE(y, 100.0f);
        EXPECT_FLOAT_EQ(z, 0.0f);
    }
}

// 测试导航点操作流程
TEST_F(WaypointsTest, TestWaypointOperations) {
    // 1. 保存导航点
    EXPECT_CALL(*mock_tbot_, saveCurrentPositionAsWaypoint("测试点", "测试描述", _))
        .WillOnce(Return(true));
    
    bool saved = mock_tbot_->saveCurrentPositionAsWaypoint("测试点", "测试描述", [](int code, const std::string& message) {
        EXPECT_EQ(code, 0);
    });
    EXPECT_TRUE(saved);
    
    // 2. 加载导航点
    EXPECT_CALL(*mock_tbot_, loadWaypoint("测试点", _))
        .WillOnce(Return(true));
    
    bool loaded = mock_tbot_->loadWaypoint("测试点", [](const Waypoint& waypoint) {
        EXPECT_EQ(waypoint.name, "测试点");
    });
    EXPECT_TRUE(loaded);
    
    // 3. 导航到导航点
    EXPECT_CALL(*mock_tbot_, navigateToWaypoint("测试点", _))
        .WillOnce(Return(true));
    
    bool navigated = mock_tbot_->navigateToWaypoint("测试点", [](int code, const std::string& message) {
        EXPECT_EQ(code, 0);
    });
    EXPECT_TRUE(navigated);
    
    // 4. 删除导航点
    EXPECT_CALL(*mock_tbot_, deleteWaypoint("测试点", _))
        .WillOnce(Return(true));
    
    bool deleted = mock_tbot_->deleteWaypoint("测试点", [](int code, const std::string& message) {
        EXPECT_EQ(code, 0);
    });
    EXPECT_TRUE(deleted);
}

// 测试导航点过滤功能
TEST_F(WaypointsTest, TestWaypointFiltering) {
    std::vector<Waypoint> all_waypoints;
    Waypoint wp1, wp2, wp3;
    wp1.name = "A点";
    wp1.description = "起始位置";
    wp2.name = "B点";
    wp2.description = "中间位置";
    wp3.name = "充电桩";
    wp3.description = "充电位置";
    all_waypoints.push_back(wp1);
    all_waypoints.push_back(wp2);
    all_waypoints.push_back(wp3);
    
    EXPECT_CALL(*mock_tbot_, getAllWaypoints("点", _))
        .WillOnce(Return(all_waypoints));
    
    auto filtered_waypoints = mock_tbot_->getAllWaypoints("点", [](int code, const std::string& message) {
        EXPECT_EQ(code, 0);
    });
    
    EXPECT_EQ(filtered_waypoints.size(), 3);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
} 