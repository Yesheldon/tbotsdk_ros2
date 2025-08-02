#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "tbot_sdk/TBotSDK.h"
#include <memory>
#include <string>

using namespace TBot;
using ::testing::_;
using ::testing::Return;
using ::testing::StrictMock;

// Mock类用于测试连接功能
class MockTBotSDK : public TBotSDK {
public:
    MockTBotSDK(const std::string& ip = "192.168.8.110") : TBotSDK(ip) {}
    
    MOCK_METHOD(bool, connect, (StatusCallback callback), (override));
    MOCK_METHOD(void, disconnect, (), (override));
    MOCK_METHOD(bool, isConnected, (), (const, override));
    MOCK_METHOD(bool, checkServiceRunning, (StatusCallback callback), (override));
};

class ConnectionTest : public ::testing::Test {
protected:
    void SetUp() override {
        mock_tbot_ = std::make_unique<StrictMock<MockTBotSDK>>("192.168.8.110");
    }
    
    void TearDown() override {
        mock_tbot_.reset();
    }
    
    std::unique_ptr<StrictMock<MockTBotSDK>> mock_tbot_;
};

// 测试连接成功
TEST_F(ConnectionTest, TestConnectSuccess) {
    EXPECT_CALL(*mock_tbot_, connect(_))
        .WillOnce(Return(true));
    
    bool result = mock_tbot_->connect([](int code, const std::string& message) {
        EXPECT_EQ(code, 0);
        EXPECT_EQ(message, "Connected successfully");
    });
    
    EXPECT_TRUE(result);
}

// 测试连接失败
TEST_F(ConnectionTest, TestConnectFailure) {
    EXPECT_CALL(*mock_tbot_, connect(_))
        .WillOnce(Return(false));
    
    bool result = mock_tbot_->connect([](int code, const std::string& message) {
        EXPECT_EQ(code, -1);
        EXPECT_EQ(message, "Failed to connect");
    });
    
    EXPECT_FALSE(result);
}

// 测试连接状态检查
TEST_F(ConnectionTest, TestIsConnected) {
    EXPECT_CALL(*mock_tbot_, isConnected())
        .WillOnce(Return(true))
        .WillOnce(Return(false));
    
    EXPECT_TRUE(mock_tbot_->isConnected());
    EXPECT_FALSE(mock_tbot_->isConnected());
}

// 测试服务状态检查
TEST_F(ConnectionTest, TestCheckServiceRunning) {
    EXPECT_CALL(*mock_tbot_, checkServiceRunning(_))
        .WillOnce(Return(true));
    
    bool result = mock_tbot_->checkServiceRunning([](int code, const std::string& message) {
        EXPECT_EQ(code, 0);
        EXPECT_EQ(message, "Service is running");
    });
    
    EXPECT_TRUE(result);
}

// 测试断开连接
TEST_F(ConnectionTest, TestDisconnect) {
    EXPECT_CALL(*mock_tbot_, disconnect())
        .Times(1);
    
    mock_tbot_->disconnect();
}

// 测试参数验证
TEST_F(ConnectionTest, TestParameterValidation) {
    // 测试无效IP地址
    auto invalid_tbot = std::make_unique<TBotSDK>("invalid_ip");
    EXPECT_FALSE(invalid_tbot->connect([](int code, const std::string& message) {
        EXPECT_EQ(code, -1);
    }));
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
} 