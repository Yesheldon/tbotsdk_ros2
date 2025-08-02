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

// Mock类用于测试数据流功能
class MockTBotSDK : public TBotSDK {
public:
    MockTBotSDK(const std::string& ip = "192.168.8.110") : TBotSDK(ip) {}
    
    MOCK_METHOD(bool, getRobotDataStream, (RobotDataCallback callback), (override));
    MOCK_METHOD(bool, getMapStream, (MapDataCallback callback), (override));
    MOCK_METHOD(bool, isConnected, (), (const, override));
};

class DataStreamTest : public ::testing::Test {
protected:
    void SetUp() override {
        mock_tbot_ = std::make_unique<StrictMock<MockTBotSDK>>("192.168.8.110");
    }
    
    void TearDown() override {
        mock_tbot_.reset();
    }
    
    std::unique_ptr<StrictMock<MockTBotSDK>> mock_tbot_;
};

// 测试机器人数据流
TEST_F(DataStreamTest, TestRobotDataStream) {
    EXPECT_CALL(*mock_tbot_, getRobotDataStream(_))
        .WillOnce(Return(true));
    
    bool result = mock_tbot_->getRobotDataStream([](const RobotData& robotData) {
        EXPECT_EQ(robotData.battery_power, 85);
        EXPECT_EQ(robotData.system_status, "normal");
        EXPECT_FLOAT_EQ(robotData.robot_pose.position.x, 1.0);
        EXPECT_FLOAT_EQ(robotData.robot_pose.position.y, 2.0);
        EXPECT_FLOAT_EQ(robotData.robot_pose.position.z, 0.0);
        EXPECT_EQ(robotData.ranges.size(), 360);
        EXPECT_FLOAT_EQ(robotData.cpu_temp, 45.5);
        EXPECT_FLOAT_EQ(robotData.localization_quality, 0.95);
    });
    
    EXPECT_TRUE(result);
}

// 测试地图数据流
TEST_F(DataStreamTest, TestMapDataStream) {
    EXPECT_CALL(*mock_tbot_, getMapStream(_))
        .WillOnce(Return(true));
    
    bool result = mock_tbot_->getMapStream([](const MapInfo& mapInfo) {
        EXPECT_FLOAT_EQ(mapInfo.resolution, 0.05);
        EXPECT_EQ(mapInfo.width, 1000);
        EXPECT_EQ(mapInfo.height, 1000);
        EXPECT_FALSE(mapInfo.data.empty());
        EXPECT_FLOAT_EQ(mapInfo.origin.position.x, -25.0);
        EXPECT_FLOAT_EQ(mapInfo.origin.position.y, -25.0);
        EXPECT_FLOAT_EQ(mapInfo.origin.position.z, 0.0);
    });
    
    EXPECT_TRUE(result);
}

// 测试机器人数据流失败
TEST_F(DataStreamTest, TestRobotDataStreamFailure) {
    EXPECT_CALL(*mock_tbot_, getRobotDataStream(_))
        .WillOnce(Return(false));
    
    bool result = mock_tbot_->getRobotDataStream([](const RobotData& robotData) {
        // 这个回调不应该被调用
        FAIL() << "Robot data callback should not be called";
    });
    
    EXPECT_FALSE(result);
}

// 测试地图数据流失败
TEST_F(DataStreamTest, TestMapDataStreamFailure) {
    EXPECT_CALL(*mock_tbot_, getMapStream(_))
        .WillOnce(Return(false));
    
    bool result = mock_tbot_->getMapStream([](const MapInfo& mapInfo) {
        // 这个回调不应该被调用
        FAIL() << "Map data callback should not be called";
    });
    
    EXPECT_FALSE(result);
}

// 测试机器人数据内容验证
TEST_F(DataStreamTest, TestRobotDataContent) {
    RobotData testData;
    testData.battery_power = 90;
    testData.system_status = "charging";
    testData.robot_pose.position.x = 5.0;
    testData.robot_pose.position.y = 3.0;
    testData.robot_pose.position.z = 0.0;
    testData.ranges = {1.0, 1.1, 1.2, 1.3, 1.4};
    testData.cpu_temp = 50.0;
    testData.localization_quality = 0.98;
    
    EXPECT_EQ(testData.battery_power, 90);
    EXPECT_EQ(testData.system_status, "charging");
    EXPECT_FLOAT_EQ(testData.robot_pose.position.x, 5.0);
    EXPECT_FLOAT_EQ(testData.robot_pose.position.y, 3.0);
    EXPECT_EQ(testData.ranges.size(), 5);
    EXPECT_FLOAT_EQ(testData.cpu_temp, 50.0);
    EXPECT_FLOAT_EQ(testData.localization_quality, 0.98);
}

// 测试地图数据内容验证
TEST_F(DataStreamTest, TestMapDataContent) {
    MapInfo testMap;
    testMap.resolution = 0.1;
    testMap.width = 500;
    testMap.height = 500;
    testMap.data = {0, 1, 2, 3, 4, 5};
    testMap.origin.position.x = -10.0;
    testMap.origin.position.y = -10.0;
    testMap.origin.position.z = 0.0;
    
    EXPECT_FLOAT_EQ(testMap.resolution, 0.1);
    EXPECT_EQ(testMap.width, 500);
    EXPECT_EQ(testMap.height, 500);
    EXPECT_EQ(testMap.data.size(), 6);
    EXPECT_FLOAT_EQ(testMap.origin.position.x, -10.0);
    EXPECT_FLOAT_EQ(testMap.origin.position.y, -10.0);
    EXPECT_FLOAT_EQ(testMap.origin.position.z, 0.0);
}

// 测试数据流连续接收
TEST_F(DataStreamTest, TestContinuousDataStream) {
    EXPECT_CALL(*mock_tbot_, getRobotDataStream(_))
        .WillOnce(Return(true))
        .WillOnce(Return(true))
        .WillOnce(Return(true));
    
    int callback_count = 0;
    auto callback = [&callback_count](const RobotData& robotData) {
        callback_count++;
        EXPECT_GT(robotData.battery_power, 0);
        EXPECT_LE(robotData.battery_power, 100);
    };
    
    EXPECT_TRUE(mock_tbot_->getRobotDataStream(callback));
    EXPECT_TRUE(mock_tbot_->getRobotDataStream(callback));
    EXPECT_TRUE(mock_tbot_->getRobotDataStream(callback));
    
    EXPECT_EQ(callback_count, 3);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
} 