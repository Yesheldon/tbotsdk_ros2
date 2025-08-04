#include <gtest/gtest.h>
#include "tbot_sdk/TBotSDK.h"
#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <thread>

using namespace TBot;

class RealDataStreamTest : public ::testing::Test {
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

// 测试机器人数据流
TEST_F(RealDataStreamTest, TestRobotDataStream) {
    bool callback_called = false;
    RobotData received_data;
    
    bool result = tbot_->getRobotDataStream([&callback_called, &received_data](const RobotData& robotData) {
        callback_called = true;
        received_data = robotData;
        
        // 验证机器人数据的基本字段
        EXPECT_GE(robotData.battery_power, 0);
        EXPECT_LE(robotData.battery_power, 100);
        EXPECT_FALSE(robotData.system_status.empty());
        
        // 验证传感器连接状态
        EXPECT_TRUE(robotData.imu_connected);
        EXPECT_TRUE(robotData.wheel_odom_connected);
        EXPECT_TRUE(robotData.lidar_connected);
        
        // 验证激光雷达数据
        EXPECT_GT(robotData.ranges.size(), 0);
        EXPECT_GE(robotData.angle_min, -3.14);
        EXPECT_LE(robotData.angle_max, 3.14);
        EXPECT_GT(robotData.angle_increment, 0);
        EXPECT_GE(robotData.range_min, 0);
        EXPECT_GT(robotData.range_max, 0);
        
        // 验证系统状态
        EXPECT_GE(robotData.cpu_temp, 0);
        EXPECT_GE(robotData.localization_quality, 0);
        EXPECT_LE(robotData.localization_quality, 1);
    });
    
    EXPECT_TRUE(result);
    
    // 等待一段时间让数据流启动
    std::this_thread::sleep_for(std::chrono::seconds(2));
    EXPECT_TRUE(callback_called);
}

// 测试地图数据流
TEST_F(RealDataStreamTest, TestMapDataStream) {
    bool callback_called = false;
    MapInfo received_map;
    
    bool result = tbot_->getMapStream([&callback_called, &received_map](const MapInfo& mapInfo) {
        callback_called = true;
        received_map = mapInfo;
        
        // 验证地图数据的基本字段
        EXPECT_GT(mapInfo.resolution, 0);
        EXPECT_GT(mapInfo.width, 0);
        EXPECT_GT(mapInfo.height, 0);
        EXPECT_FALSE(mapInfo.data.empty());
        
        // 验证地图原点
        EXPECT_FINITE(mapInfo.origin.position.x);
        EXPECT_FINITE(mapInfo.origin.position.y);
        EXPECT_FINITE(mapInfo.origin.position.z);
    });
    
    EXPECT_TRUE(result);
    
    // 等待一段时间让数据流启动
    std::this_thread::sleep_for(std::chrono::seconds(2));
    EXPECT_TRUE(callback_called);
}

// 测试机器人数据内容验证
TEST_F(RealDataStreamTest, TestRobotDataContent) {
    bool data_received = false;
    
    tbot_->getRobotDataStream([&data_received](const RobotData& robotData) {
        data_received = true;
        
        // 验证电池电量范围
        EXPECT_GE(robotData.battery_power, 0);
        EXPECT_LE(robotData.battery_power, 100);
        
        // 验证充电模式
        EXPECT_GE(robotData.charging_mode, 0);
        EXPECT_LE(robotData.charging_mode, 2);
        
        // 验证机器人位姿
        EXPECT_FINITE(robotData.robot_pose.position.x);
        EXPECT_FINITE(robotData.robot_pose.position.y);
        EXPECT_FINITE(robotData.robot_pose.position.z);
        EXPECT_FINITE(robotData.robot_pose.orientation.x);
        EXPECT_FINITE(robotData.robot_pose.orientation.y);
        EXPECT_FINITE(robotData.robot_pose.orientation.z);
        EXPECT_FINITE(robotData.robot_pose.orientation.w);
        
        // 验证激光雷达位姿
        EXPECT_FINITE(robotData.laser_pose.position.x);
        EXPECT_FINITE(robotData.laser_pose.position.y);
        EXPECT_FINITE(robotData.laser_pose.position.z);
        
        // 验证路径数据
        EXPECT_GE(robotData.path_poses.size(), 0);
        
        // 验证超声波数据
        EXPECT_GE(robotData.sonar_data.size(), 0);
        
        // 验证速度数据
        EXPECT_GE(robotData.current_speed.size(), 0);
        EXPECT_GE(robotData.max_speed.size(), 0);
        
        // 验证碰撞监控状态
        // 这个字段应该是布尔值
    });
    
    // 等待数据接收
    std::this_thread::sleep_for(std::chrono::seconds(3));
    EXPECT_TRUE(data_received);
}

// 测试地图数据内容验证
TEST_F(RealDataStreamTest, TestMapDataContent) {
    bool map_received = false;
    
    tbot_->getMapStream([&map_received](const MapInfo& mapInfo) {
        map_received = true;
        
        // 验证地图分辨率
        EXPECT_GT(mapInfo.resolution, 0);
        EXPECT_LT(mapInfo.resolution, 1.0); // 分辨率应该小于1米
        
        // 验证地图尺寸
        EXPECT_GT(mapInfo.width, 0);
        EXPECT_GT(mapInfo.height, 0);
        EXPECT_LE(mapInfo.width, 10000); // 地图宽度不应该过大
        EXPECT_LE(mapInfo.height, 10000); // 地图高度不应该过大
        
        // 验证地图数据
        EXPECT_FALSE(mapInfo.data.empty());
        EXPECT_EQ(mapInfo.data.size(), mapInfo.width * mapInfo.height);
        
        // 验证地图原点
        EXPECT_FINITE(mapInfo.origin.position.x);
        EXPECT_FINITE(mapInfo.origin.position.y);
        EXPECT_FINITE(mapInfo.origin.position.z);
        EXPECT_FINITE(mapInfo.origin.orientation.x);
        EXPECT_FINITE(mapInfo.origin.orientation.y);
        EXPECT_FINITE(mapInfo.origin.orientation.z);
        EXPECT_FINITE(mapInfo.origin.orientation.w);
    });
    
    // 等待地图数据接收
    std::this_thread::sleep_for(std::chrono::seconds(3));
    EXPECT_TRUE(map_received);
}

// 测试数据流连续接收
TEST_F(RealDataStreamTest, TestContinuousDataStream) {
    int callback_count = 0;
    auto start_time = std::chrono::steady_clock::now();
    
    tbot_->getRobotDataStream([&callback_count](const RobotData& robotData) {
        callback_count++;
        
        // 验证每次接收的数据都是有效的
        EXPECT_GE(robotData.battery_power, 0);
        EXPECT_LE(robotData.battery_power, 100);
        EXPECT_GT(robotData.ranges.size(), 0);
    });
    
    // 等待5秒钟，应该接收到多次数据
    std::this_thread::sleep_for(std::chrono::seconds(5));
    
    // 验证在5秒内至少接收到几次数据
    EXPECT_GT(callback_count, 0);
    
    auto end_time = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time);
    std::cout << "Received " << callback_count << " data packets in " << duration.count() << " seconds" << std::endl;
}

// 测试数据流错误处理
TEST_F(RealDataStreamTest, TestDataStreamErrorHandling) {
    // 测试在未连接状态下获取数据流
    auto disconnected_tbot = std::make_unique<TBotSDK>("192.168.8.110");
    
    bool result = disconnected_tbot->getRobotDataStream([](const RobotData& robotData) {
        FAIL() << "Should not receive data when not connected";
    });
    
    EXPECT_FALSE(result);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
} 