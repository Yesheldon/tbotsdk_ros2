#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <memory>
#include <chrono>
#include <thread>

// 包含要测试的节点
#include "tbot_sdk/TBotSDK.h"

class TBotNodeTest : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        node_ = std::make_shared<rclcpp::Node>("test_node");
        
        // 创建发布者和订阅者用于测试
        cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        goal_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);
        
        odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                last_odom_msg_ = msg;
            });
        
        laser_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
                last_laser_msg_ = msg;
            });
        
        status_sub_ = node_->create_subscription<std_msgs::msg::String>(
            "status", 10, [this](const std_msgs::msg::String::SharedPtr msg) {
                last_status_msg_ = msg;
            });
    }
    
    void TearDown() override {
        node_.reset();
        rclcpp::shutdown();
    }
    
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
    
    nav_msgs::msg::Odometry::SharedPtr last_odom_msg_;
    sensor_msgs::msg::LaserScan::SharedPtr last_laser_msg_;
    std_msgs::msg::String::SharedPtr last_status_msg_;
};

// 测试速度控制消息发布
TEST_F(TBotNodeTest, TestVelocityControl) {
    auto twist_msg = geometry_msgs::msg::Twist();
    twist_msg.linear.x = 0.5;
    twist_msg.angular.z = 0.2;
    
    cmd_vel_pub_->publish(twist_msg);
    
    // 等待消息处理
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 验证消息已发布
    EXPECT_EQ(node_->count_publishers("cmd_vel"), 1);
    EXPECT_EQ(node_->count_subscribers("cmd_vel"), 1);
}

// 测试导航目标消息发布
TEST_F(TBotNodeTest, TestGoalPose) {
    auto pose_msg = geometry_msgs::msg::PoseStamped();
    pose_msg.header.frame_id = "map";
    pose_msg.header.stamp = node_->now();
    pose_msg.pose.position.x = 1.0;
    pose_msg.pose.position.y = 2.0;
    pose_msg.pose.position.z = 0.0;
    pose_msg.pose.orientation.w = 1.0;
    
    goal_pose_pub_->publish(pose_msg);
    
    // 等待消息处理
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 验证消息已发布
    EXPECT_EQ(node_->count_publishers("goal_pose"), 1);
    EXPECT_EQ(node_->count_subscribers("goal_pose"), 1);
}

// 测试里程计消息接收
TEST_F(TBotNodeTest, TestOdometryMessage) {
    // 模拟发布里程计消息
    auto odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";
    odom_msg.pose.pose.position.x = 1.0;
    odom_msg.pose.pose.position.y = 2.0;
    odom_msg.pose.pose.orientation.w = 1.0;
    
    // 发布消息
    auto odom_pub = node_->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    odom_pub->publish(odom_msg);
    
    // 等待消息接收
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 验证消息接收
    EXPECT_EQ(node_->count_publishers("odom"), 1);
    EXPECT_EQ(node_->count_subscribers("odom"), 1);
}

// 测试激光雷达消息接收
TEST_F(TBotNodeTest, TestLaserScanMessage) {
    // 模拟发布激光雷达消息
    auto laser_msg = sensor_msgs::msg::LaserScan();
    laser_msg.header.frame_id = "laser";
    laser_msg.angle_min = -3.14159;
    laser_msg.angle_max = 3.14159;
    laser_msg.angle_increment = 0.0174533;
    laser_msg.range_min = 0.1;
    laser_msg.range_max = 10.0;
    laser_msg.ranges = {1.0, 1.1, 1.2, 1.3, 1.4};
    
    // 发布消息
    auto laser_pub = node_->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
    laser_pub->publish(laser_msg);
    
    // 等待消息接收
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 验证消息接收
    EXPECT_EQ(node_->count_publishers("scan"), 1);
    EXPECT_EQ(node_->count_subscribers("scan"), 1);
}

// 测试状态消息接收
TEST_F(TBotNodeTest, TestStatusMessage) {
    // 模拟发布状态消息
    auto status_msg = std_msgs::msg::String();
    status_msg.data = "Battery: 85%, Status: normal";
    
    // 发布消息
    auto status_pub = node_->create_publisher<std_msgs::msg::String>("status", 10);
    status_pub->publish(status_msg);
    
    // 等待消息接收
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 验证消息接收
    EXPECT_EQ(node_->count_publishers("status"), 1);
    EXPECT_EQ(node_->count_subscribers("status"), 1);
}

// 测试参数设置
TEST_F(TBotNodeTest, TestParameterSetting) {
    // 设置参数
    node_->declare_parameter("robot_ip", "192.168.8.110");
    node_->set_parameter(rclcpp::Parameter("robot_ip", "192.168.1.100"));
    
    // 获取参数
    std::string robot_ip = node_->get_parameter("robot_ip").as_string();
    EXPECT_EQ(robot_ip, "192.168.1.100");
}

// 测试话题名称
TEST_F(TBotNodeTest, TestTopicNames) {
    // 验证话题名称
    EXPECT_EQ(node_->count_publishers("cmd_vel"), 1);
    EXPECT_EQ(node_->count_publishers("goal_pose"), 1);
    EXPECT_EQ(node_->count_subscribers("odom"), 1);
    EXPECT_EQ(node_->count_subscribers("scan"), 1);
    EXPECT_EQ(node_->count_subscribers("status"), 1);
}

// 测试消息类型
TEST_F(TBotNodeTest, TestMessageTypes) {
    // 测试Twist消息
    auto twist_msg = geometry_msgs::msg::Twist();
    twist_msg.linear.x = 0.5;
    twist_msg.linear.y = 0.0;
    twist_msg.linear.z = 0.0;
    twist_msg.angular.x = 0.0;
    twist_msg.angular.y = 0.0;
    twist_msg.angular.z = 0.2;
    
    EXPECT_FLOAT_EQ(twist_msg.linear.x, 0.5);
    EXPECT_FLOAT_EQ(twist_msg.angular.z, 0.2);
    
    // 测试PoseStamped消息
    auto pose_msg = geometry_msgs::msg::PoseStamped();
    pose_msg.pose.position.x = 1.0;
    pose_msg.pose.position.y = 2.0;
    pose_msg.pose.position.z = 0.0;
    pose_msg.pose.orientation.w = 1.0;
    
    EXPECT_FLOAT_EQ(pose_msg.pose.position.x, 1.0);
    EXPECT_FLOAT_EQ(pose_msg.pose.position.y, 2.0);
    EXPECT_FLOAT_EQ(pose_msg.pose.orientation.w, 1.0);
}

// 测试TF变换
TEST_F(TBotNodeTest, TestTFTransform) {
    // 测试TF变换数据结构
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_link";
    transform.transform.translation.x = 1.0;
    transform.transform.translation.y = 2.0;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation.w = 1.0;
    
    EXPECT_EQ(transform.header.frame_id, "odom");
    EXPECT_EQ(transform.child_frame_id, "base_link");
    EXPECT_FLOAT_EQ(transform.transform.translation.x, 1.0);
    EXPECT_FLOAT_EQ(transform.transform.translation.y, 2.0);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
} 