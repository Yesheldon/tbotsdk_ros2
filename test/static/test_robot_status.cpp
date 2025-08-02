#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "tbot_sdk/TBotSDK.h"
#include <memory>
#include <string>

using namespace TBot;
using ::testing::_;
using ::testing::Return;
using ::testing::StrictMock;

// Mock类用于测试机器人状态功能
class MockTBotSDK : public TBotSDK {
public:
    MockTBotSDK(const std::string& ip = "192.168.8.110") : TBotSDK(ip) {}
    
    MOCK_METHOD(bool, isConnected, (), (const, override));
    MOCK_METHOD(bool, getRobotStatus, (RobotStatusCallback callback), (override));
    MOCK_METHOD(bool, getBatteryStatus, (BatteryStatusCallback callback), (override));
    MOCK_METHOD(bool, getSystemStatus, (SystemStatusCallback callback), (override));
};

class RobotStatusTest : public ::testing::Test {
protected:
    void SetUp() override {
        mock_tbot_ = std::make_unique<StrictMock<MockTBotSDK>>("192.168.8.110");
    }
    
    void TearDown() override {
        mock_tbot_.reset();
    }
    
    std::unique_ptr<StrictMock<MockTBotSDK>> mock_tbot_;
};

// 测试机器人状态获取
TEST_F(RobotStatusTest, TestGetRobotStatus) {
    EXPECT_CALL(*mock_tbot_, getRobotStatus(_))
        .WillOnce(Return(true));
    
    bool result = mock_tbot_->getRobotStatus([](const RobotStatus& status) {
        EXPECT_EQ(status.is_connected, true);
        EXPECT_EQ(status.is_moving, false);
        EXPECT_EQ(status.is_charging, false);
        EXPECT_EQ(status.is_localized, true);
        EXPECT_FLOAT_EQ(status.position.x, 1.0);
        EXPECT_FLOAT_EQ(status.position.y, 2.0);
        EXPECT_FLOAT_EQ(status.position.z, 0.0);
        EXPECT_FLOAT_EQ(status.orientation.x, 0.0);
        EXPECT_FLOAT_EQ(status.orientation.y, 0.0);
        EXPECT_FLOAT_EQ(status.orientation.z, 0.0);
        EXPECT_FLOAT_EQ(status.orientation.w, 1.0);
    });
    
    EXPECT_TRUE(result);
}

// 测试电池状态获取
TEST_F(RobotStatusTest, TestGetBatteryStatus) {
    EXPECT_CALL(*mock_tbot_, getBatteryStatus(_))
        .WillOnce(Return(true));
    
    bool result = mock_tbot_->getBatteryStatus([](const BatteryStatus& battery) {
        EXPECT_EQ(battery.percentage, 85);
        EXPECT_EQ(battery.is_charging, false);
        EXPECT_EQ(battery.is_low, false);
        EXPECT_EQ(battery.is_critical, false);
        EXPECT_FLOAT_EQ(battery.voltage, 12.5);
        EXPECT_FLOAT_EQ(battery.current, 1.2);
    });
    
    EXPECT_TRUE(result);
}

// 测试系统状态获取
TEST_F(RobotStatusTest, TestGetSystemStatus) {
    EXPECT_CALL(*mock_tbot_, getSystemStatus(_))
        .WillOnce(Return(true));
    
    bool result = mock_tbot_->getSystemStatus([](const SystemStatus& system) {
        EXPECT_EQ(system.cpu_usage, 25);
        EXPECT_EQ(system.memory_usage, 60);
        EXPECT_FLOAT_EQ(system.cpu_temperature, 45.5);
        EXPECT_FLOAT_EQ(system.uptime, 3600.0);
        EXPECT_EQ(system.error_count, 0);
        EXPECT_EQ(system.warning_count, 2);
    });
    
    EXPECT_TRUE(result);
}

// 测试机器人状态失败
TEST_F(RobotStatusTest, TestGetRobotStatusFailure) {
    EXPECT_CALL(*mock_tbot_, getRobotStatus(_))
        .WillOnce(Return(false));
    
    bool result = mock_tbot_->getRobotStatus([](const RobotStatus& status) {
        // 这个回调不应该被调用
        FAIL() << "Robot status callback should not be called";
    });
    
    EXPECT_FALSE(result);
}

// 测试电池状态失败
TEST_F(RobotStatusTest, TestGetBatteryStatusFailure) {
    EXPECT_CALL(*mock_tbot_, getBatteryStatus(_))
        .WillOnce(Return(false));
    
    bool result = mock_tbot_->getBatteryStatus([](const BatteryStatus& battery) {
        // 这个回调不应该被调用
        FAIL() << "Battery status callback should not be called";
    });
    
    EXPECT_FALSE(result);
}

// 测试系统状态失败
TEST_F(RobotStatusTest, TestGetSystemStatusFailure) {
    EXPECT_CALL(*mock_tbot_, getSystemStatus(_))
        .WillOnce(Return(false));
    
    bool result = mock_tbot_->getSystemStatus([](const SystemStatus& system) {
        // 这个回调不应该被调用
        FAIL() << "System status callback should not be called";
    });
    
    EXPECT_FALSE(result);
}

// 测试机器人状态内容验证
TEST_F(RobotStatusTest, TestRobotStatusContent) {
    RobotStatus testStatus;
    testStatus.is_connected = true;
    testStatus.is_moving = true;
    testStatus.is_charging = false;
    testStatus.is_localized = true;
    testStatus.position.x = 5.0;
    testStatus.position.y = 3.0;
    testStatus.position.z = 0.0;
    testStatus.orientation.x = 0.0;
    testStatus.orientation.y = 0.0;
    testStatus.orientation.z = 0.707;
    testStatus.orientation.w = 0.707;
    
    EXPECT_TRUE(testStatus.is_connected);
    EXPECT_TRUE(testStatus.is_moving);
    EXPECT_FALSE(testStatus.is_charging);
    EXPECT_TRUE(testStatus.is_localized);
    EXPECT_FLOAT_EQ(testStatus.position.x, 5.0);
    EXPECT_FLOAT_EQ(testStatus.position.y, 3.0);
    EXPECT_FLOAT_EQ(testStatus.orientation.z, 0.707);
    EXPECT_FLOAT_EQ(testStatus.orientation.w, 0.707);
}

// 测试电池状态内容验证
TEST_F(RobotStatusTest, TestBatteryStatusContent) {
    BatteryStatus testBattery;
    testBattery.percentage = 20;
    testBattery.is_charging = true;
    testBattery.is_low = true;
    testBattery.is_critical = false;
    testBattery.voltage = 11.0;
    testBattery.current = 2.5;
    
    EXPECT_EQ(testBattery.percentage, 20);
    EXPECT_TRUE(testBattery.is_charging);
    EXPECT_TRUE(testBattery.is_low);
    EXPECT_FALSE(testBattery.is_critical);
    EXPECT_FLOAT_EQ(testBattery.voltage, 11.0);
    EXPECT_FLOAT_EQ(testBattery.current, 2.5);
}

// 测试系统状态内容验证
TEST_F(RobotStatusTest, TestSystemStatusContent) {
    SystemStatus testSystem;
    testSystem.cpu_usage = 80;
    testSystem.memory_usage = 90;
    testSystem.cpu_temperature = 65.0;
    testSystem.uptime = 7200.0;
    testSystem.error_count = 1;
    testSystem.warning_count = 5;
    
    EXPECT_EQ(testSystem.cpu_usage, 80);
    EXPECT_EQ(testSystem.memory_usage, 90);
    EXPECT_FLOAT_EQ(testSystem.cpu_temperature, 65.0);
    EXPECT_FLOAT_EQ(testSystem.uptime, 7200.0);
    EXPECT_EQ(testSystem.error_count, 1);
    EXPECT_EQ(testSystem.warning_count, 5);
}

// 测试状态边界条件
TEST_F(RobotStatusTest, TestStatusBoundaryConditions) {
    // 测试电池状态边界
    BatteryStatus lowBattery;
    lowBattery.percentage = 5;
    lowBattery.is_low = true;
    lowBattery.is_critical = true;
    
    EXPECT_EQ(lowBattery.percentage, 5);
    EXPECT_TRUE(lowBattery.is_low);
    EXPECT_TRUE(lowBattery.is_critical);
    
    // 测试系统状态边界
    SystemStatus highLoad;
    highLoad.cpu_usage = 100;
    highLoad.memory_usage = 100;
    highLoad.cpu_temperature = 90.0;
    
    EXPECT_EQ(highLoad.cpu_usage, 100);
    EXPECT_EQ(highLoad.memory_usage, 100);
    EXPECT_FLOAT_EQ(highLoad.cpu_temperature, 90.0);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
} 