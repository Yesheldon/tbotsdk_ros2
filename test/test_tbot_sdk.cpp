#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "tbot_sdk/TBotSDK.h"
#include <memory>
#include <string>

using namespace TBot;
using ::testing::_;
using ::testing::Return;
using ::testing::StrictMock;

// Mock类用于测试
class MockTBotSDK : public TBotSDK {
public:
    MockTBotSDK(const std::string& ip = "192.168.8.110") : TBotSDK(ip) {}
    
    MOCK_METHOD(bool, connect, (StatusCallback callback), (override));
    MOCK_METHOD(void, disconnect, (), (override));
    MOCK_METHOD(bool, isConnected, (), (const, override));
    MOCK_METHOD(bool, checkServiceRunning, (StatusCallback callback), (override));
    MOCK_METHOD(std::vector<std::string>, getMapList, (StatusCallback callback), (override));
    MOCK_METHOD(bool, startNavigation, (const std::string& mapName, StatusCallback callback), (override));
    MOCK_METHOD(bool, navigateTo, (float x, float y, float z, float w, StatusCallback callback), (override));
    MOCK_METHOD(bool, velocityControl, (float linearSpeed, float angularSpeed, StatusCallback callback), (override));
    MOCK_METHOD(bool, getRobotDataStream, (RobotDataCallback callback), (override));
    MOCK_METHOD(bool, getMapStream, (MapDataCallback callback), (override));
};

class TBotSDKTest : public ::testing::Test {
protected:
    void SetUp() override {
        mock_tbot_ = std::make_unique<StrictMock<MockTBotSDK>>("192.168.8.110");
    }
    
    void TearDown() override {
        mock_tbot_.reset();
    }
    
    std::unique_ptr<StrictMock<MockTBotSDK>> mock_tbot_;
};

// 测试连接功能
TEST_F(TBotSDKTest, TestConnect) {
    EXPECT_CALL(*mock_tbot_, connect(_))
        .WillOnce(Return(true));
    
    bool result = mock_tbot_->connect([](int code, const std::string& message) {
        EXPECT_EQ(code, 0);
        EXPECT_EQ(message, "Connected successfully");
    });
    
    EXPECT_TRUE(result);
}

// 测试连接失败
TEST_F(TBotSDKTest, TestConnectFailure) {
    EXPECT_CALL(*mock_tbot_, connect(_))
        .WillOnce(Return(false));
    
    bool result = mock_tbot_->connect([](int code, const std::string& message) {
        EXPECT_EQ(code, -1);
        EXPECT_EQ(message, "Failed to connect");
    });
    
    EXPECT_FALSE(result);
}

// 测试服务状态检查
TEST_F(TBotSDKTest, TestCheckServiceRunning) {
    EXPECT_CALL(*mock_tbot_, checkServiceRunning(_))
        .WillOnce(Return(true));
    
    bool result = mock_tbot_->checkServiceRunning([](int code, const std::string& message) {
        EXPECT_EQ(code, 0);
        EXPECT_EQ(message, "Service is running");
    });
    
    EXPECT_TRUE(result);
}

// 测试获取地图列表
TEST_F(TBotSDKTest, TestGetMapList) {
    std::vector<std::string> expected_maps = {"map1", "map2", "map3"};
    
    EXPECT_CALL(*mock_tbot_, getMapList(_))
        .WillOnce(Return(expected_maps));
    
    auto maps = mock_tbot_->getMapList([](int code, const std::string& message) {
        EXPECT_EQ(code, 0);
        EXPECT_EQ(message, "Success");
    });
    
    EXPECT_EQ(maps.size(), 3);
    EXPECT_EQ(maps[0], "map1");
    EXPECT_EQ(maps[1], "map2");
    EXPECT_EQ(maps[2], "map3");
}

// 测试启动导航
TEST_F(TBotSDKTest, TestStartNavigation) {
    EXPECT_CALL(*mock_tbot_, startNavigation("test_map", _))
        .WillOnce(Return(true));
    
    bool result = mock_tbot_->startNavigation("test_map", [](int code, const std::string& message) {
        EXPECT_EQ(code, 0);
        EXPECT_EQ(message, "Navigation started");
    });
    
    EXPECT_TRUE(result);
}

// 测试导航到指定位置
TEST_F(TBotSDKTest, TestNavigateTo) {
    EXPECT_CALL(*mock_tbot_, navigateTo(1.0f, 2.0f, 0.0f, 0.5f, _))
        .WillOnce(Return(true));
    
    bool result = mock_tbot_->navigateTo(1.0f, 2.0f, 0.0f, 0.5f, [](int code, const std::string& message) {
        EXPECT_EQ(code, 0);
        EXPECT_EQ(message, "Navigation goal accepted");
    });
    
    EXPECT_TRUE(result);
}

// 测试速度控制
TEST_F(TBotSDKTest, TestVelocityControl) {
    EXPECT_CALL(*mock_tbot_, velocityControl(0.5f, 0.2f, _))
        .WillOnce(Return(true));
    
    bool result = mock_tbot_->velocityControl(0.5f, 0.2f, [](int code, const std::string& message) {
        EXPECT_EQ(code, 0);
        EXPECT_EQ(message, "Velocity control successful");
    });
    
    EXPECT_TRUE(result);
}

// 测试机器人数据流
TEST_F(TBotSDKTest, TestRobotDataStream) {
    EXPECT_CALL(*mock_tbot_, getRobotDataStream(_))
        .WillOnce(Return(true));
    
    bool result = mock_tbot_->getRobotDataStream([](const RobotData& robotData) {
        EXPECT_EQ(robotData.battery_power, 85);
        EXPECT_EQ(robotData.system_status, "normal");
    });
    
    EXPECT_TRUE(result);
}

// 测试地图数据流
TEST_F(TBotSDKTest, TestMapDataStream) {
    EXPECT_CALL(*mock_tbot_, getMapStream(_))
        .WillOnce(Return(true));
    
    bool result = mock_tbot_->getMapStream([](const MapInfo& mapInfo) {
        EXPECT_EQ(mapInfo.resolution, 0.05f);
        EXPECT_EQ(mapInfo.width, 1000);
        EXPECT_EQ(mapInfo.height, 1000);
    });
    
    EXPECT_TRUE(result);
}

// 测试连接状态
TEST_F(TBotSDKTest, TestIsConnected) {
    EXPECT_CALL(*mock_tbot_, isConnected())
        .WillOnce(Return(true))
        .WillOnce(Return(false));
    
    EXPECT_TRUE(mock_tbot_->isConnected());
    EXPECT_FALSE(mock_tbot_->isConnected());
}

// 测试断开连接
TEST_F(TBotSDKTest, TestDisconnect) {
    EXPECT_CALL(*mock_tbot_, disconnect())
        .Times(1);
    
    mock_tbot_->disconnect();
}

// 测试参数验证
TEST_F(TBotSDKTest, TestParameterValidation) {
    // 测试无效IP地址
    auto invalid_tbot = std::make_unique<TBotSDK>("invalid_ip");
    EXPECT_FALSE(invalid_tbot->connect([](int code, const std::string& message) {
        EXPECT_EQ(code, -1);
    }));
}

// 测试回调函数
TEST_F(TBotSDKTest, TestCallbackFunctions) {
    bool callback_called = false;
    std::string callback_message;
    
    auto callback = [&callback_called, &callback_message](int code, const std::string& message) {
        callback_called = true;
        callback_message = message;
    };
    
    EXPECT_CALL(*mock_tbot_, connect(_))
        .WillOnce(::testing::Invoke([&callback](StatusCallback cb) {
            cb(0, "Test message");
            return true;
        }));
    
    mock_tbot_->connect(callback);
    
    EXPECT_TRUE(callback_called);
    EXPECT_EQ(callback_message, "Test message");
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
} 