#include <gtest/gtest.h>
#include "tbot_sdk/TBotSDK.h"
#include <memory>
#include <string>
#include <chrono>
#include <thread>

using namespace TBot;

class RealRobotStatusTest : public ::testing::Test {
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

// 测试机器人状态获取
TEST_F(RealRobotStatusTest, TestGetRobotStatus) {
    bool callback_called = false;
    
    bool result = tbot_->getRobotStatus([&callback_called](const RobotStatus& status) {
        callback_called = true;
        
        // 验证机器人状态的基本字段
        EXPECT_TRUE(status.is_connected);
        EXPECT_FINITE(status.position.x);
        EXPECT_FINITE(status.position.y);
        EXPECT_FINITE(status.position.z);
        EXPECT_FINITE(status.orientation.x);
        EXPECT_FINITE(status.orientation.y);
        EXPECT_FINITE(status.orientation.z);
        EXPECT_FINITE(status.orientation.w);
        
        // 验证位姿四元数的模长接近1
        float quaternion_norm = status.orientation.x * status.orientation.x +
                               status.orientation.y * status.orientation.y +
                               status.orientation.z * status.orientation.z +
                               status.orientation.w * status.orientation.w;
        EXPECT_NEAR(quaternion_norm, 1.0, 0.1);
        
        std::cout << "Robot position: (" << status.position.x << ", " 
                  << status.position.y << ", " << status.position.z << ")" << std::endl;
        std::cout << "Robot orientation: (" << status.orientation.x << ", " 
                  << status.orientation.y << ", " << status.orientation.z 
                  << ", " << status.orientation.w << ")" << std::endl;
        std::cout << "Is moving: " << (status.is_moving ? "true" : "false") << std::endl;
        std::cout << "Is charging: " << (status.is_charging ? "true" : "false") << std::endl;
        std::cout << "Is localized: " << (status.is_localized ? "true" : "false") << std::endl;
    });
    
    EXPECT_TRUE(result);
    EXPECT_TRUE(callback_called);
}

// 测试电池状态获取
TEST_F(RealRobotStatusTest, TestGetBatteryStatus) {
    bool callback_called = false;
    
    bool result = tbot_->getBatteryStatus([&callback_called](const BatteryStatus& battery) {
        callback_called = true;
        
        // 验证电池状态的基本字段
        EXPECT_GE(battery.percentage, 0);
        EXPECT_LE(battery.percentage, 100);
        EXPECT_GE(battery.voltage, 0);
        EXPECT_GE(battery.current, -50); // 充电时电流可能为负值
        EXPECT_LE(battery.current, 50);
        
        // 验证电池状态的一致性
        if (battery.percentage <= 10) {
            EXPECT_TRUE(battery.is_low);
        }
        if (battery.percentage <= 5) {
            EXPECT_TRUE(battery.is_critical);
        }
        if (battery.is_charging) {
            EXPECT_LE(battery.current, 0); // 充电时电流应该为负值或接近0
        }
        
        std::cout << "Battery percentage: " << battery.percentage << "%" << std::endl;
        std::cout << "Battery voltage: " << battery.voltage << "V" << std::endl;
        std::cout << "Battery current: " << battery.current << "A" << std::endl;
        std::cout << "Is charging: " << (battery.is_charging ? "true" : "false") << std::endl;
        std::cout << "Is low: " << (battery.is_low ? "true" : "false") << std::endl;
        std::cout << "Is critical: " << (battery.is_critical ? "true" : "false") << std::endl;
    });
    
    EXPECT_TRUE(result);
    EXPECT_TRUE(callback_called);
}

// 测试系统状态获取
TEST_F(RealRobotStatusTest, TestGetSystemStatus) {
    bool callback_called = false;
    
    bool result = tbot_->getSystemStatus([&callback_called](const SystemStatus& system) {
        callback_called = true;
        
        // 验证系统状态的基本字段
        EXPECT_GE(system.cpu_usage, 0);
        EXPECT_LE(system.cpu_usage, 100);
        EXPECT_GE(system.memory_usage, 0);
        EXPECT_LE(system.memory_usage, 100);
        EXPECT_GE(system.cpu_temperature, 0);
        EXPECT_LE(system.cpu_temperature, 100); // CPU温度不应该超过100度
        EXPECT_GE(system.uptime, 0);
        EXPECT_GE(system.error_count, 0);
        EXPECT_GE(system.warning_count, 0);
        
        // 验证系统状态的一致性
        if (system.cpu_temperature > 80) {
            std::cout << "Warning: High CPU temperature: " << system.cpu_temperature << "°C" << std::endl;
        }
        if (system.cpu_usage > 90) {
            std::cout << "Warning: High CPU usage: " << system.cpu_usage << "%" << std::endl;
        }
        if (system.memory_usage > 90) {
            std::cout << "Warning: High memory usage: " << system.memory_usage << "%" << std::endl;
        }
        
        std::cout << "CPU usage: " << system.cpu_usage << "%" << std::endl;
        std::cout << "Memory usage: " << system.memory_usage << "%" << std::endl;
        std::cout << "CPU temperature: " << system.cpu_temperature << "°C" << std::endl;
        std::cout << "Uptime: " << system.uptime << " seconds" << std::endl;
        std::cout << "Error count: " << system.error_count << std::endl;
        std::cout << "Warning count: " << system.warning_count << std::endl;
    });
    
    EXPECT_TRUE(result);
    EXPECT_TRUE(callback_called);
}

// 测试机器人状态内容验证
TEST_F(RealRobotStatusTest, TestRobotStatusContent) {
    bool status_received = false;
    
    tbot_->getRobotStatus([&status_received](const RobotStatus& status) {
        status_received = true;
        
        // 验证机器人位姿的合理性
        EXPECT_FINITE(status.position.x);
        EXPECT_FINITE(status.position.y);
        EXPECT_FINITE(status.position.z);
        
        // 验证机器人位姿范围（假设机器人在合理的工作空间内）
        EXPECT_GE(status.position.x, -100);
        EXPECT_LE(status.position.x, 100);
        EXPECT_GE(status.position.y, -100);
        EXPECT_LE(status.position.y, 100);
        EXPECT_NEAR(status.position.z, 0.0, 1.0); // Z轴应该接近0
        
        // 验证四元数归一化
        float quaternion_norm = status.orientation.x * status.orientation.x +
                               status.orientation.y * status.orientation.y +
                               status.orientation.z * status.orientation.z +
                               status.orientation.w * status.orientation.w;
        EXPECT_NEAR(quaternion_norm, 1.0, 0.1);
        
        // 验证状态标志的一致性
        if (status.is_moving) {
            // 如果机器人正在移动，应该已经定位
            EXPECT_TRUE(status.is_localized);
        }
    });
    
    EXPECT_TRUE(status_received);
}

// 测试电池状态内容验证
TEST_F(RealRobotStatusTest, TestBatteryStatusContent) {
    bool battery_received = false;
    
    tbot_->getBatteryStatus([&battery_received](const BatteryStatus& battery) {
        battery_received = true;
        
        // 验证电池电量的合理性
        EXPECT_GE(battery.percentage, 0);
        EXPECT_LE(battery.percentage, 100);
        
        // 验证电池电压的合理性（假设是12V系统）
        EXPECT_GE(battery.voltage, 10.0);
        EXPECT_LE(battery.voltage, 15.0);
        
        // 验证电池电流的合理性
        EXPECT_GE(battery.current, -20.0); // 充电电流
        EXPECT_LE(battery.current, 20.0);  // 放电电流
        
        // 验证电池状态标志的一致性
        if (battery.percentage <= 5) {
            EXPECT_TRUE(battery.is_critical);
        }
        if (battery.percentage <= 10) {
            EXPECT_TRUE(battery.is_low);
        }
        if (battery.is_critical) {
            EXPECT_TRUE(battery.is_low);
        }
    });
    
    EXPECT_TRUE(battery_received);
}

// 测试系统状态内容验证
TEST_F(RealRobotStatusTest, TestSystemStatusContent) {
    bool system_received = false;
    
    tbot_->getSystemStatus([&system_received](const SystemStatus& system) {
        system_received = true;
        
        // 验证CPU使用率的合理性
        EXPECT_GE(system.cpu_usage, 0);
        EXPECT_LE(system.cpu_usage, 100);
        
        // 验证内存使用率的合理性
        EXPECT_GE(system.memory_usage, 0);
        EXPECT_LE(system.memory_usage, 100);
        
        // 验证CPU温度的合理性
        EXPECT_GE(system.cpu_temperature, 0);
        EXPECT_LE(system.cpu_temperature, 100);
        
        // 验证运行时间的合理性
        EXPECT_GE(system.uptime, 0);
        
        // 验证错误和警告计数的合理性
        EXPECT_GE(system.error_count, 0);
        EXPECT_GE(system.warning_count, 0);
        
        // 验证系统状态的一致性
        if (system.cpu_temperature > 80) {
            std::cout << "Warning: High CPU temperature detected" << std::endl;
        }
        if (system.cpu_usage > 90) {
            std::cout << "Warning: High CPU usage detected" << std::endl;
        }
        if (system.memory_usage > 90) {
            std::cout << "Warning: High memory usage detected" << std::endl;
        }
    });
    
    EXPECT_TRUE(system_received);
}

// 测试状态边界条件
TEST_F(RealRobotStatusTest, TestStatusBoundaryConditions) {
    // 测试连续获取状态
    int status_count = 0;
    auto start_time = std::chrono::steady_clock::now();
    
    tbot_->getRobotStatus([&status_count](const RobotStatus& status) {
        status_count++;
        
        // 验证每次获取的状态都是有效的
        EXPECT_TRUE(status.is_connected);
        EXPECT_FINITE(status.position.x);
        EXPECT_FINITE(status.position.y);
        EXPECT_FINITE(status.position.z);
    });
    
    // 等待一段时间，应该能获取到状态更新
    std::this_thread::sleep_for(std::chrono::seconds(3));
    
    EXPECT_GT(status_count, 0);
    
    auto end_time = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time);
    std::cout << "Received " << status_count << " status updates in " 
              << duration.count() << " seconds" << std::endl;
}

// 测试状态错误处理
TEST_F(RealRobotStatusTest, TestStatusErrorHandling) {
    // 测试在未连接状态下获取状态
    auto disconnected_tbot = std::make_unique<TBotSDK>("192.168.8.110");
    
    bool result = disconnected_tbot->getRobotStatus([](const RobotStatus& status) {
        FAIL() << "Should not receive status when not connected";
    });
    
    EXPECT_FALSE(result);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
} 