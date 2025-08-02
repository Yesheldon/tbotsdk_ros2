#include "tbot_sdk/TBotSDK.h"
#include <iostream>
#include <chrono>
#include <thread>

using namespace TBot;

void testWaypoints() {
    TBotSDK tbot;
    
    // 连接机器人
    std::cout << "=== 连接机器人 ===" << std::endl;
    bool connected = tbot.connect([](int code, const std::string& msg) {
        std::cout << "连接结果: " << code << " - " << msg << std::endl;
    });
    
    if (!connected) {
        std::cout << "连接失败，跳过后续测试" << std::endl;
        return;
    }
    
    // 测试保存当前位置作为位置点
    std::cout << "\n=== 保存当前位置作为A点 ===" << std::endl;
    bool savedA = tbot.saveCurrentPositionAsWaypoint("A点", "起始位置", [](int code, const std::string& msg) {
        std::cout << "保存A点结果: " << code << " - " << msg << std::endl;
    });
    
    // 移动机器人到新位置
    std::cout << "\n=== 移动机器人到新位置 ===" << std::endl;
    tbot.navigateTo(1.0f, 1.0f, 0.0f, 1.0f, [](int code, const std::string& msg) {
        std::cout << "移动结果: " << code << " - " << msg << std::endl;
    });
    
    // 等待移动完成
    std::this_thread::sleep_for(std::chrono::seconds(5));
    
    // 保存当前位置作为B点
    std::cout << "\n=== 保存当前位置作为B点 ===" << std::endl;
    bool savedB = tbot.saveCurrentPositionAsWaypoint("B点", "中间位置", [](int code, const std::string& msg) {
        std::cout << "保存B点结果: " << code << " - " << msg << std::endl;
    });
    
    // 再移动机器人
    std::cout << "\n=== 移动机器人到第三个位置 ===" << std::endl;
    tbot.navigateTo(2.0f, 2.0f, 0.0f, 1.0f, [](int code, const std::string& msg) {
        std::cout << "移动结果: " << code << " - " << msg << std::endl;
    });
    
    // 等待移动完成
    std::this_thread::sleep_for(std::chrono::seconds(5));
    
    // 保存当前位置作为C点
    std::cout << "\n=== 保存当前位置作为C点 ===" << std::endl;
    bool savedC = tbot.saveCurrentPositionAsWaypoint("C点", "终点位置", [](int code, const std::string& msg) {
        std::cout << "保存C点结果: " << code << " - " << msg << std::endl;
    });
    
    // 测试加载位置点
    std::cout << "\n=== 加载A点信息 ===" << std::endl;
    tbot.loadWaypoint("A点", [](const Waypoint& waypoint) {
        if (!waypoint.name.empty()) {
            std::cout << "A点信息:" << std::endl;
            std::cout << "  名称: " << waypoint.name << std::endl;
            std::cout << "  位置: (" << waypoint.pose.position.x << ", " 
                     << waypoint.pose.position.y << ", " << waypoint.pose.position.z << ")" << std::endl;
            std::cout << "  描述: " << waypoint.description << std::endl;
            std::cout << "  创建时间: " << waypoint.created_time << std::endl;
        } else {
            std::cout << "A点不存在" << std::endl;
        }
    });
    
    // 测试获取所有位置点
    std::cout << "\n=== 获取所有位置点 ===" << std::endl;
    std::vector<Waypoint> waypoints = tbot.getAllWaypoints("", [](int code, const std::string& msg) {
        std::cout << "获取位置点列表结果: " << code << " - " << msg << std::endl;
    });
    
    std::cout << "找到 " << waypoints.size() << " 个位置点:" << std::endl;
    for (const auto& wp : waypoints) {
        std::cout << "  " << wp.name << ": " << wp.description 
                 << " (" << wp.pose.position.x << ", " << wp.pose.position.y << ")" << std::endl;
    }
    
    // 测试导航到位置点
    std::cout << "\n=== 导航到A点 ===" << std::endl;
    bool navigatedToA = tbot.navigateToWaypoint("A点", [](int code, const std::string& msg) {
        std::cout << "导航到A点结果: " << code << " - " << msg << std::endl;
    });
    
    // 等待导航完成
    std::this_thread::sleep_for(std::chrono::seconds(3));
    
    // 测试同步导航到位置点
    std::cout << "\n=== 同步导航到B点 ===" << std::endl;
    bool navigatedToB = tbot.navigateToWaypointAndWait("B点", 30, [](int code, const std::string& msg) {
        std::cout << "同步导航到B点结果: " << code << " - " << msg << std::endl;
    });
    
    // 测试删除位置点
    std::cout << "\n=== 删除C点 ===" << std::endl;
    bool deletedC = tbot.deleteWaypoint("C点", [](int code, const std::string& msg) {
        std::cout << "删除C点结果: " << code << " - " << msg << std::endl;
    });
    
    // 验证删除结果
    std::cout << "\n=== 验证C点是否已删除 ===" << std::endl;
    tbot.loadWaypoint("C点", [](const Waypoint& waypoint) {
        if (waypoint.name.empty()) {
            std::cout << "C点已成功删除" << std::endl;
        } else {
            std::cout << "C点仍然存在" << std::endl;
        }
    });
}

int main() {
    std::cout << "开始测试位置点管理功能..." << std::endl;
    testWaypoints();
    std::cout << "位置点管理功能测试完成" << std::endl;
    return 0;
} 