#include "tbot_sdk/TBotSDK.h"
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

using namespace TBot;
using namespace std::chrono_literals;

class WaypointService : public rclcpp::Node {
public:
    WaypointService() : Node("waypoint_service") {
        // 获取参数
        this->declare_parameter("robot_ip", "192.168.8.110");
        std::string robot_ip = this->get_parameter("robot_ip").as_string();
        
        // 初始化SDK
        m_tbot = std::make_unique<TBotSDK>(robot_ip);
        
        // 连接机器人
        if (!m_tbot->connect([](int code, const std::string& msg) {
            RCLCPP_INFO(this->get_logger(), "连接结果: %d - %s", code, msg.c_str());
        })) {
            RCLCPP_ERROR(this->get_logger(), "无法连接到机器人");
            return;
        }
        
        // 创建服务
        m_save_waypoint_service = this->create_service<std_srvs::srv::Trigger>(
            "save_waypoint", 
            std::bind(&WaypointService::handleSaveWaypoint, this, 
                     std::placeholders::_1, std::placeholders::_2));
        
        m_load_waypoint_service = this->create_service<std_srvs::srv::Trigger>(
            "load_waypoint", 
            std::bind(&WaypointService::handleLoadWaypoint, this, 
                     std::placeholders::_1, std::placeholders::_2));
        
        m_navigate_to_waypoint_service = this->create_service<std_srvs::srv::Trigger>(
            "navigate_to_waypoint", 
            std::bind(&WaypointService::handleNavigateToWaypoint, this, 
                     std::placeholders::_1, std::placeholders::_2));
        
        m_delete_waypoint_service = this->create_service<std_srvs::srv::Trigger>(
            "delete_waypoint", 
            std::bind(&WaypointService::handleDeleteWaypoint, this, 
                     std::placeholders::_1, std::placeholders::_2));
        
        // 创建发布者
        m_waypoint_list_pub = this->create_publisher<std_msgs::msg::String>("waypoint_list", 10);
        
        // 创建定时器，定期发布位置点列表
        m_timer = this->create_wall_timer(10s, std::bind(&WaypointService::publishWaypointList, this));
        
        RCLCPP_INFO(this->get_logger(), "位置点服务已启动");
    }
    
private:
    void handleSaveWaypoint(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        
        // 获取当前时间作为位置点名称
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << "Waypoint_" << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S");
        std::string waypoint_name = ss.str();
        
        // 保存当前位置作为位置点
        bool success = m_tbot->saveCurrentPositionAsWaypoint(waypoint_name, "自动保存的位置点", 
            [&](int code, const std::string& msg) {
                if (code == 0) {
                    response->success = true;
                    response->message = "位置点保存成功: " + waypoint_name;
                    RCLCPP_INFO(this->get_logger(), "位置点保存成功: %s", waypoint_name.c_str());
                } else {
                    response->success = false;
                    response->message = "位置点保存失败: " + msg;
                    RCLCPP_ERROR(this->get_logger(), "位置点保存失败: %s", msg.c_str());
                }
            });
        
        if (!success) {
            response->success = false;
            response->message = "保存位置点操作失败";
        }
    }
    
    void handleLoadWaypoint(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        
        // 获取所有位置点
        std::vector<Waypoint> waypoints = m_tbot->getAllWaypoints("", 
            [&](int code, const std::string& msg) {
                if (code != 0) {
                    RCLCPP_ERROR(this->get_logger(), "获取位置点列表失败: %s", msg.c_str());
                }
            });
        
        if (waypoints.empty()) {
            response->success = false;
            response->message = "没有找到任何位置点";
        } else {
            response->success = true;
            response->message = "找到 " + std::to_string(waypoints.size()) + " 个位置点";
            
            // 显示位置点信息
            for (const auto& wp : waypoints) {
                RCLCPP_INFO(this->get_logger(), "位置点: %s - %s (%.2f, %.2f)", 
                           wp.name.c_str(), wp.description.c_str(), 
                           wp.pose.position.x, wp.pose.position.y);
            }
        }
    }
    
    void handleNavigateToWaypoint(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        
        // 这里简化处理，导航到第一个找到的位置点
        std::vector<Waypoint> waypoints = m_tbot->getAllWaypoints("", nullptr);
        
        if (waypoints.empty()) {
            response->success = false;
            response->message = "没有找到任何位置点";
            return;
        }
        
        // 导航到第一个位置点
        std::string target_waypoint = waypoints[0].name;
        bool success = m_tbot->navigateToWaypoint(target_waypoint, 
            [&](int code, const std::string& msg) {
                if (code == 0) {
                    response->success = true;
                    response->message = "开始导航到位置点: " + target_waypoint;
                    RCLCPP_INFO(this->get_logger(), "开始导航到位置点: %s", target_waypoint.c_str());
                } else {
                    response->success = false;
                    response->message = "导航失败: " + msg;
                    RCLCPP_ERROR(this->get_logger(), "导航失败: %s", msg.c_str());
                }
            });
        
        if (!success) {
            response->success = false;
            response->message = "导航操作失败";
        }
    }
    
    void handleDeleteWaypoint(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        
        // 这里简化处理，删除第一个找到的位置点
        std::vector<Waypoint> waypoints = m_tbot->getAllWaypoints("", nullptr);
        
        if (waypoints.empty()) {
            response->success = false;
            response->message = "没有找到任何位置点";
            return;
        }
        
        // 删除第一个位置点
        std::string target_waypoint = waypoints[0].name;
        bool success = m_tbot->deleteWaypoint(target_waypoint, 
            [&](int code, const std::string& msg) {
                if (code == 0) {
                    response->success = true;
                    response->message = "位置点删除成功: " + target_waypoint;
                    RCLCPP_INFO(this->get_logger(), "位置点删除成功: %s", target_waypoint.c_str());
                } else {
                    response->success = false;
                    response->message = "删除失败: " + msg;
                    RCLCPP_ERROR(this->get_logger(), "删除失败: %s", msg.c_str());
                }
            });
        
        if (!success) {
            response->success = false;
            response->message = "删除操作失败";
        }
    }
    
    void publishWaypointList() {
        std::vector<Waypoint> waypoints = m_tbot->getAllWaypoints("", nullptr);
        
        std::string waypoint_list;
        for (const auto& wp : waypoints) {
            waypoint_list += wp.name + ":" + wp.description + ";";
        }
        
        auto msg = std_msgs::msg::String();
        msg.data = waypoint_list;
        m_waypoint_list_pub->publish(msg);
    }
    
    std::unique_ptr<TBotSDK> m_tbot;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_save_waypoint_service;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_load_waypoint_service;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_navigate_to_waypoint_service;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_delete_waypoint_service;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_waypoint_list_pub;
    rclcpp::TimerBase::SharedPtr m_timer;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointService>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 