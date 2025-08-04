#include <gtest/gtest.h>
#include "tbot_sdk/TBotSDK.h"
#include <memory>
#include <string>
#include <chrono>
#include <thread>

using namespace TBot;

class RealVelocityControlTest : public ::testing::Test {
protected:
    void SetUp() override {
        tbot_ = std::make_unique<TBotSDK>("192.168.8.110");
        tbot_->connect();
    }
    
    void TearDown() override {
        // 停止所有运动
        if (tbot_->isConnected()) {
            tbot_->stopMovement();
            tbot_->disconnect();
        }
        tbot_.reset();
    }
    
    std::unique_ptr<TBotSDK> tbot_;
};

// 测试设置最大速度
TEST_F(RealVelocityControlTest, TestSetMaxSpeed) {
    // 设置最大线速度和角速度
    bool result = tbot_->setMaxSpeed(0.5f, 0.3f);
    EXPECT_TRUE(result);
    
    std::cout << "Max speed set: linear=0.5m/s, angular=0.3rad/s" << std::endl;
}

// 测试速度控制
TEST_F(RealVelocityControlTest, TestVelocityControl) {
    // 设置最大速度
    tbot_->setMaxSpeed(0.3f, 0.2f);
    
    // 向前移动
    bool result = tbot_->velocityControl(0.2f, 0.0f);
    EXPECT_TRUE(result);
    
    std::cout << "Moving forward at 0.2m/s" << std::endl;
    
    // 等待2秒
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    // 停止运动
    tbot_->stopMovement();
    std::this_thread::sleep_for(std::chrono::seconds(1));
}

// 测试转向控制
TEST_F(RealVelocityControlTest, TestTurnControl) {
    // 设置最大速度
    tbot_->setMaxSpeed(0.3f, 0.2f);
    
    // 原地转向
    bool result = tbot_->velocityControl(0.0f, 0.1f);
    EXPECT_TRUE(result);
    
    std::cout << "Turning at 0.1rad/s" << std::endl;
    
    // 等待2秒
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    // 停止运动
    tbot_->stopMovement();
    std::this_thread::sleep_for(std::chrono::seconds(1));
}

// 测试停止运动
TEST_F(RealVelocityControlTest, TestStopMovement) {
    // 先设置一个运动
    tbot_->setMaxSpeed(0.3f, 0.2f);
    tbot_->velocityControl(0.1f, 0.0f);
    
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    // 停止运动
    bool result = tbot_->stopMovement();
    EXPECT_TRUE(result);
    
    std::cout << "Movement stopped" << std::endl;
}

// 测试移动动作
TEST_F(RealVelocityControlTest, TestMoveAction) {
    // 测试前进动作
    bool result = tbot_->move("forward", 0.5f, 0.2f, 5);
    EXPECT_TRUE(result);
    
    std::cout << "Move action: forward 0.5m at 0.2m/s" << std::endl;
    
    // 等待动作完成
    std::this_thread::sleep_for(std::chrono::seconds(6));
}

// 测试参数验证
TEST_F(RealVelocityControlTest, TestParameterValidation) {
    // 测试无效速度参数
    bool result = tbot_->velocityControl(999.0f, 999.0f);
    EXPECT_FALSE(result);
    
    // 测试无效移动参数
    bool move_result = tbot_->move("invalid_action", 1.0f, 1.0f, 1);
    EXPECT_FALSE(move_result);
}

// 测试速度范围
TEST_F(RealVelocityControlTest, TestSpeedRange) {
    // 测试低速
    bool low_speed = tbot_->velocityControl(0.1f, 0.05f);
    EXPECT_TRUE(low_speed);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    tbot_->stopMovement();
    
    // 测试中速
    bool mid_speed = tbot_->velocityControl(0.3f, 0.15f);
    EXPECT_TRUE(mid_speed);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    tbot_->stopMovement();
    
    // 测试高速（在安全范围内）
    bool high_speed = tbot_->velocityControl(0.5f, 0.25f);
    EXPECT_TRUE(high_speed);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    tbot_->stopMovement();
}

// 测试连续运动控制
TEST_F(RealVelocityControlTest, TestContinuousControl) {
    tbot_->setMaxSpeed(0.4f, 0.3f);
    
    // 连续改变速度
    for (int i = 0; i < 5; i++) {
        float linear_speed = 0.1f + i * 0.05f;
        float angular_speed = 0.05f + i * 0.02f;
        
        bool result = tbot_->velocityControl(linear_speed, angular_speed);
        EXPECT_TRUE(result);
        
        std::cout << "Speed: linear=" << linear_speed << ", angular=" << angular_speed << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    
    // 停止运动
    tbot_->stopMovement();
}

// 测试错误处理
TEST_F(RealVelocityControlTest, TestErrorHandling) {
    // 测试在未连接状态下控制速度
    auto disconnected_tbot = std::make_unique<TBotSDK>("192.168.8.110");
    
    bool result = disconnected_tbot->velocityControl(0.1f, 0.0f);
    EXPECT_FALSE(result);
    
    bool move_result = disconnected_tbot->move("forward", 1.0f, 0.2f, 5);
    EXPECT_FALSE(move_result);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
} 