#include "tbot_sdk/TBotSDK.h"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <memory>
#include <string>

class TBotNode : public rclcpp::Node
{
public:
    TBotNode() : Node("tbot_node")
    {
        // 获取参数
        this->declare_parameter("robot_ip", "192.168.8.110");
        std::string robot_ip = this->get_parameter("robot_ip").as_string();
        
        // 初始化SDK
        tbot_ = std::make_unique<TBot::TBotSDK>(robot_ip);
        
        // 创建发布者
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        laser_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
        status_pub_ = this->create_publisher<std_msgs::msg::String>("status", 10);
        
        // 创建订阅者
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&TBotNode::cmdVelCallback, this, std::placeholders::_1));
        
        goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "goal_pose", 10, std::bind(&TBotNode::goalPoseCallback, this, std::placeholders::_1));
        
        // 创建TF广播器
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
        
        // 创建定时器
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&TBotNode::timerCallback, this));
        
        // 连接机器人
        if (!tbot_->connect([this](int code, const std::string& message) {
            RCLCPP_INFO(this->get_logger(), "TBot Status: %d - %s", code, message.c_str());
        })) {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to TBot");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "TBot node initialized successfully");
    }
    
    ~TBotNode()
    {
        if (tbot_) {
            tbot_->disconnect();
        }
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (tbot_ && tbot_->isConnected()) {
            tbot_->velocityControl(msg->linear.x, msg->angular.z, [this](int code, const std::string& message) {
                if (code != 0) {
                    RCLCPP_WARN(this->get_logger(), "Velocity control failed: %s", message.c_str());
                }
            });
        }
    }
    
    void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (tbot_ && tbot_->isConnected()) {
            // 获取四元数
            tf2::Quaternion q(
                msg->pose.orientation.x,
                msg->pose.orientation.y,
                msg->pose.orientation.z,
                msg->pose.orientation.w
            );
            
            // 转换为欧拉角
            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
            
            tbot_->navigateTo(
                msg->pose.position.x,
                msg->pose.position.y,
                msg->pose.position.z,
                yaw,
                [this](int code, const std::string& message) {
                    if (code == 0) {
                        RCLCPP_INFO(this->get_logger(), "Navigation goal accepted");
                    } else {
                        RCLCPP_WARN(this->get_logger(), "Navigation failed: %s", message.c_str());
                    }
                }
            );
        }
    }
    
    void timerCallback()
    {
        if (!tbot_ || !tbot_->isConnected()) {
            return;
        }
        
        // 获取机器人数据
        tbot_->getRobotDataStream([this](const TBot::RobotData& robotData) {
            // 发布里程计数据
            auto odom_msg = nav_msgs::msg::Odometry();
            odom_msg.header.stamp = this->now();
            odom_msg.header.frame_id = "odom";
            odom_msg.child_frame_id = "base_link";
            
            odom_msg.pose.pose.position.x = robotData.robot_pose.position.x;
            odom_msg.pose.pose.position.y = robotData.robot_pose.position.y;
            odom_msg.pose.pose.position.z = robotData.robot_pose.position.z;
            
            odom_msg.pose.pose.orientation.x = robotData.robot_pose.orientation.x;
            odom_msg.pose.pose.orientation.y = robotData.robot_pose.orientation.y;
            odom_msg.pose.pose.orientation.z = robotData.robot_pose.orientation.z;
            odom_msg.pose.pose.orientation.w = robotData.robot_pose.orientation.w;
            
            odom_pub_->publish(odom_msg);
            
            // 发布TF变换
            geometry_msgs::msg::TransformStamped transform;
            transform.header.stamp = this->now();
            transform.header.frame_id = "odom";
            transform.child_frame_id = "base_link";
            transform.transform.translation.x = robotData.robot_pose.position.x;
            transform.transform.translation.y = robotData.robot_pose.position.y;
            transform.transform.translation.z = robotData.robot_pose.position.z;
            transform.transform.rotation.x = robotData.robot_pose.orientation.x;
            transform.transform.rotation.y = robotData.robot_pose.orientation.y;
            transform.transform.rotation.z = robotData.robot_pose.orientation.z;
            transform.transform.rotation.w = robotData.robot_pose.orientation.w;
            
            tf_broadcaster_->sendTransform(transform);
            
            // 发布激光雷达数据
            if (!robotData.ranges.empty()) {
                auto laser_msg = sensor_msgs::msg::LaserScan();
                laser_msg.header.stamp = this->now();
                laser_msg.header.frame_id = "laser";
                laser_msg.angle_min = robotData.angle_min;
                laser_msg.angle_max = robotData.angle_max;
                laser_msg.angle_increment = robotData.angle_increment;
                laser_msg.range_min = robotData.range_min;
                laser_msg.range_max = robotData.range_max;
                laser_msg.ranges = robotData.ranges;
                
                laser_pub_->publish(laser_msg);
            }
            
            // 发布状态信息
            auto status_msg = std_msgs::msg::String();
            status_msg.data = "Battery: " + std::to_string(robotData.battery_power) + 
                            "%, Status: " + robotData.system_status;
            status_pub_->publish(status_msg);
        });
    }
    
    std::unique_ptr<TBot::TBotSDK> tbot_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TBotNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 