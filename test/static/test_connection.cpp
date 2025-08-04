#include <gtest/gtest.h>
#include "tbot_sdk/TBotSDK.h"
#include <memory>
#include <string>

using namespace TBot;

class RealConnectionTest : public ::testing::Test {
protected:
    void SetUp() override {
        // 使用真实的TBotSDK，连接真实的机器人
        tbot_ = std::make_unique<TBotSDK>("192.168.8.110");
    }
    
    void TearDown() override {
        if (tbot_->isConnected()) {
            tbot_->disconnect();
        }
        tbot_.reset();
    }
    
    std::unique_ptr<TBotSDK> tbot_;
};

// 测试真实连接成功
TEST_F(RealConnectionTest, TestConnectSuccess) {
    bool result = tbot_->connect();
    EXPECT_TRUE(result);
    EXPECT_TRUE(tbot_->isConnected());
}

// 测试连接失败（使用无效IP）
TEST_F(RealConnectionTest, TestConnectFailure) {
    auto invalid_tbot = std::make_unique<TBotSDK>("192.168.1.999");
    bool result = invalid_tbot->connect();
    EXPECT_FALSE(result);
}

// 测试连接状态检查
TEST_F(RealConnectionTest, TestIsConnected) {
    // 初始状态应该是未连接
    EXPECT_FALSE(tbot_->isConnected());
    
    // 连接后应该是已连接
    tbot_->connect();
    EXPECT_TRUE(tbot_->isConnected());
    
    // 断开后应该是未连接
    tbot_->disconnect();
    EXPECT_FALSE(tbot_->isConnected());
}

// 测试服务状态检查
TEST_F(RealConnectionTest, TestCheckServiceRunning) {
    tbot_->connect();
    bool result = tbot_->checkServiceRunning();
    EXPECT_TRUE(result);
}

// 测试断开连接
TEST_F(RealConnectionTest, TestDisconnect) {
    tbot_->connect();
    EXPECT_TRUE(tbot_->isConnected());
    
    tbot_->disconnect();
    EXPECT_FALSE(tbot_->isConnected());
}

// 测试参数验证
TEST_F(RealConnectionTest, TestParameterValidation) {
    // 测试无效IP地址
    auto invalid_tbot = std::make_unique<TBotSDK>("invalid_ip");
    bool result = invalid_tbot->connect();
    EXPECT_FALSE(result);
}

// 测试回调函数
TEST_F(RealConnectionTest, TestCallbackFunction) {
    bool callback_called = false;
    int callback_code = -1;
    std::string callback_message;
    
    bool result = tbot_->connect([&callback_called, &callback_code, &callback_message](int code, const std::string& message) {
        callback_called = true;
        callback_code = code;
        callback_message = message;
    });
    
    EXPECT_TRUE(result);
    EXPECT_TRUE(callback_called);
    EXPECT_EQ(callback_code, 0);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
} 