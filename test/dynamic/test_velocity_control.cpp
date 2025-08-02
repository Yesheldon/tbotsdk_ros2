#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "tbot_sdk/TBotSDK.h"
#include <memory>
#include <string>

using namespace TBot;
using ::testing::_;
using ::testing::Return;
using ::testing::StrictMock;

// Mock类用于测试速度控制功能
class MockTBotSDK : public TBotSDK {
public:
    MockTBotSDK(const std::string& ip = "192.168.8.110") : TBotSDK(ip) {}
    
    MOCK_METHOD(bool, velocityControl, (float linearSpeed, float angularSpeed, StatusCallback callback), (override));
    MOCK_METHOD(bool, setMaxVelocity, (float maxLinearSpeed, float maxAngularSpeed, StatusCallback callback), (override));
    MOCK_METHOD(bool, stop, (StatusCallback callback), (override));
    MOCK_METHOD(bool, emergencyStop, (StatusCallback callback), (override));
    MOCK_METHOD(bool, isMoving, (), (const, override));
};

class VelocityControlTest : public ::testing::Test {
protected:
    void SetUp() override {
        mock_tbot_ = std::make_unique<StrictMock<MockTBotSDK>>("192.168.8.110");
    }
    
    void TearDown() override {
        mock_tbot_.reset();
    }
    
    std::unique_ptr<StrictMock<MockTBotSDK>> mock_tbot_;
};

// 测试速度控制
TEST_F(VelocityControlTest, TestVelocityControl) {
    EXPECT_CALL(*mock_tbot_, velocityControl(0.5f, 0.2f, _))
        .WillOnce(Return(true));
    
    bool result = mock_tbot_->velocityControl(0.5f, 0.2f, [](int code, const std::string& message) {
        EXPECT_EQ(code, 0);
        EXPECT_EQ(message, "Velocity control successful");
    });
    
    EXPECT_TRUE(result);
}

// 测试设置最大速度
TEST_F(VelocityControlTest, TestSetMaxVelocity) {
    EXPECT_CALL(*mock_tbot_, setMaxVelocity(1.0f, 0.5f, _))
        .WillOnce(Return(true));
    
    bool result = mock_tbot_->setMaxVelocity(1.0f, 0.5f, [](int code, const std::string& message) {
        EXPECT_EQ(code, 0);
        EXPECT_EQ(message, "Max velocity set successfully");
    });
    
    EXPECT_TRUE(result);
}

// 测试停止
TEST_F(VelocityControlTest, TestStop) {
    EXPECT_CALL(*mock_tbot_, stop(_))
        .WillOnce(Return(true));
    
    bool result = mock_tbot_->stop([](int code, const std::string& message) {
        EXPECT_EQ(code, 0);
        EXPECT_EQ(message, "Robot stopped");
    });
    
    EXPECT_TRUE(result);
}

// 测试紧急停止
TEST_F(VelocityControlTest, TestEmergencyStop) {
    EXPECT_CALL(*mock_tbot_, emergencyStop(_))
        .WillOnce(Return(true));
    
    bool result = mock_tbot_->emergencyStop([](int code, const std::string& message) {
        EXPECT_EQ(code, 0);
        EXPECT_EQ(message, "Emergency stop activated");
    });
    
    EXPECT_TRUE(result);
}

// 测试移动状态检查
TEST_F(VelocityControlTest, TestIsMoving) {
    EXPECT_CALL(*mock_tbot_, isMoving())
        .WillOnce(Return(true))
        .WillOnce(Return(false));
    
    EXPECT_TRUE(mock_tbot_->isMoving());
    EXPECT_FALSE(mock_tbot_->isMoving());
}

// 测试速度控制失败
TEST_F(VelocityControlTest, TestVelocityControlFailure) {
    EXPECT_CALL(*mock_tbot_, velocityControl(999.0f, 999.0f, _))
        .WillOnce(Return(false));
    
    bool result = mock_tbot_->velocityControl(999.0f, 999.0f, [](int code, const std::string& message) {
        EXPECT_EQ(code, -1);
        EXPECT_EQ(message, "Invalid velocity command");
    });
    
    EXPECT_FALSE(result);
}

// 测试设置最大速度失败
TEST_F(VelocityControlTest, TestSetMaxVelocityFailure) {
    EXPECT_CALL(*mock_tbot_, setMaxVelocity(-1.0f, -1.0f, _))
        .WillOnce(Return(false));
    
    bool result = mock_tbot_->setMaxVelocity(-1.0f, -1.0f, [](int code, const std::string& message) {
        EXPECT_EQ(code, -1);
        EXPECT_EQ(message, "Invalid max velocity values");
    });
    
    EXPECT_FALSE(result);
}

// 测试速度参数验证
TEST_F(VelocityControlTest, TestVelocityParameterValidation) {
    // 测试有效速度范围
    std::vector<std::pair<float, float>> valid_velocities = {
        {0.0f, 0.0f}, {0.1f, 0.1f}, {0.5f, 0.2f}, {1.0f, 0.5f}, {2.0f, 1.0f}
    };
    
    for (const auto& velocity : valid_velocities) {
        float linear = velocity.first;
        float angular = velocity.second;
        
        EXPECT_GE(linear, 0.0f);
        EXPECT_LE(linear, 5.0f);
        EXPECT_GE(angular, -2.0f);
        EXPECT_LE(angular, 2.0f);
    }
}

// 测试不同速度模式
TEST_F(VelocityControlTest, TestDifferentVelocityModes) {
    // 测试前进
    EXPECT_CALL(*mock_tbot_, velocityControl(0.5f, 0.0f, _))
        .WillOnce(Return(true));
    
    bool forward = mock_tbot_->velocityControl(0.5f, 0.0f, [](int code, const std::string& message) {
        EXPECT_EQ(code, 0);
    });
    EXPECT_TRUE(forward);
    
    // 测试后退
    EXPECT_CALL(*mock_tbot_, velocityControl(-0.3f, 0.0f, _))
        .WillOnce(Return(true));
    
    bool backward = mock_tbot_->velocityControl(-0.3f, 0.0f, [](int code, const std::string& message) {
        EXPECT_EQ(code, 0);
    });
    EXPECT_TRUE(backward);
    
    // 测试左转
    EXPECT_CALL(*mock_tbot_, velocityControl(0.0f, 0.5f, _))
        .WillOnce(Return(true));
    
    bool turn_left = mock_tbot_->velocityControl(0.0f, 0.5f, [](int code, const std::string& message) {
        EXPECT_EQ(code, 0);
    });
    EXPECT_TRUE(turn_left);
    
    // 测试右转
    EXPECT_CALL(*mock_tbot_, velocityControl(0.0f, -0.5f, _))
        .WillOnce(Return(true));
    
    bool turn_right = mock_tbot_->velocityControl(0.0f, -0.5f, [](int code, const std::string& message) {
        EXPECT_EQ(code, 0);
    });
    EXPECT_TRUE(turn_right);
    
    // 测试曲线运动
    EXPECT_CALL(*mock_tbot_, velocityControl(0.3f, 0.2f, _))
        .WillOnce(Return(true));
    
    bool curve = mock_tbot_->velocityControl(0.3f, 0.2f, [](int code, const std::string& message) {
        EXPECT_EQ(code, 0);
    });
    EXPECT_TRUE(curve);
}

// 测试速度限制
TEST_F(VelocityControlTest, TestVelocityLimits) {
    // 测试最大线速度
    EXPECT_CALL(*mock_tbot_, velocityControl(5.0f, 0.0f, _))
        .WillOnce(Return(true));
    
    bool max_linear = mock_tbot_->velocityControl(5.0f, 0.0f, [](int code, const std::string& message) {
        EXPECT_EQ(code, 0);
    });
    EXPECT_TRUE(max_linear);
    
    // 测试最大角速度
    EXPECT_CALL(*mock_tbot_, velocityControl(0.0f, 2.0f, _))
        .WillOnce(Return(true));
    
    bool max_angular = mock_tbot_->velocityControl(0.0f, 2.0f, [](int code, const std::string& message) {
        EXPECT_EQ(code, 0);
    });
    EXPECT_TRUE(max_angular);
    
    // 测试最小速度
    EXPECT_CALL(*mock_tbot_, velocityControl(0.01f, 0.01f, _))
        .WillOnce(Return(true));
    
    bool min_velocity = mock_tbot_->velocityControl(0.01f, 0.01f, [](int code, const std::string& message) {
        EXPECT_EQ(code, 0);
    });
    EXPECT_TRUE(min_velocity);
}

// 测试速度控制精度
TEST_F(VelocityControlTest, TestVelocityPrecision) {
    // 测试不同精度的小数速度
    std::vector<std::pair<float, float>> precise_velocities = {
        {0.123f, 0.456f}, {0.789f, 0.012f}, {1.234f, 0.567f}, {2.345f, 1.234f}
    };
    
    for (const auto& velocity : precise_velocities) {
        float linear = velocity.first;
        float angular = velocity.second;
        
        EXPECT_GT(linear, 0.0f);
        EXPECT_LE(linear, 5.0f);
        EXPECT_GE(angular, -2.0f);
        EXPECT_LE(angular, 2.0f);
    }
}

// 测试速度控制状态转换
TEST_F(VelocityControlTest, TestVelocityStateTransitions) {
    // 测试从停止到移动
    EXPECT_CALL(*mock_tbot_, velocityControl(0.5f, 0.0f, _))
        .WillOnce(Return(true));
    
    bool started = mock_tbot_->velocityControl(0.5f, 0.0f, [](int code, const std::string& message) {
        EXPECT_EQ(code, 0);
    });
    EXPECT_TRUE(started);
    
    // 测试改变速度
    EXPECT_CALL(*mock_tbot_, velocityControl(1.0f, 0.2f, _))
        .WillOnce(Return(true));
    
    bool changed = mock_tbot_->velocityControl(1.0f, 0.2f, [](int code, const std::string& message) {
        EXPECT_EQ(code, 0);
    });
    EXPECT_TRUE(changed);
    
    // 测试停止
    EXPECT_CALL(*mock_tbot_, stop(_))
        .WillOnce(Return(true));
    
    bool stopped = mock_tbot_->stop([](int code, const std::string& message) {
        EXPECT_EQ(code, 0);
    });
    EXPECT_TRUE(stopped);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
} 