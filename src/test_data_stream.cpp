#include "tbot_sdk/TBotSDK.h"
#include <iostream>
#include <thread>
#include <chrono>

void statusCallback(int code, const std::string& message) {
    std::cout << "Status: " << code << " - " << message << std::endl;
}

void mapDataCallback(const TBot::MapInfo& mapInfo) {
    std::cout << "Map Data Received:" << std::endl;
    std::cout << "  Resolution: " << mapInfo.resolution << std::endl;
    std::cout << "  Size: " << mapInfo.width << "x" << mapInfo.height << std::endl;
    std::cout << "  Data points: " << mapInfo.data.size() << std::endl;
    std::cout << "  Origin: (" << mapInfo.origin.position.x << ", " 
              << mapInfo.origin.position.y << ", " << mapInfo.origin.position.z << ")" << std::endl;
}

void robotDataCallback(const TBot::RobotData& robotData) {
    std::cout << "Robot Data Received:" << std::endl;
    std::cout << "  Battery: " << robotData.battery_power << "%" << std::endl;
    std::cout << "  Status: " << robotData.system_status << std::endl;
    std::cout << "  Position: (" << robotData.robot_pose.position.x << ", " 
              << robotData.robot_pose.position.y << ", " << robotData.robot_pose.position.z << ")" << std::endl;
    std::cout << "  Laser ranges: " << robotData.ranges.size() << " points" << std::endl;
    std::cout << "  CPU Temp: " << robotData.cpu_temp << "°C" << std::endl;
    std::cout << "  Localization Quality: " << robotData.localization_quality << std::endl;
}

int main() {
    std::cout << "TBot Data Stream Test" << std::endl;
    std::cout << "=====================" << std::endl;
    
    // 创建SDK实例
    TBot::TBotSDK tbot("192.168.8.110");
    
    // 连接机器人
    if (!tbot.connect(statusCallback)) {
        std::cerr << "Failed to connect to TBot" << std::endl;
        return 1;
    }
    
    std::cout << "Connected to TBot successfully!" << std::endl;
    
    // 测试地图数据流
    std::cout << "\nTesting map data stream..." << std::endl;
    if (tbot.getMapStream(mapDataCallback)) {
        std::cout << "Map data stream test passed" << std::endl;
    } else {
        std::cout << "Map data stream test failed" << std::endl;
    }
    
    // 测试机器人数据流
    std::cout << "\nTesting robot data stream..." << std::endl;
    if (tbot.getRobotDataStream(robotDataCallback)) {
        std::cout << "Robot data stream test passed" << std::endl;
    } else {
        std::cout << "Robot data stream test failed" << std::endl;
    }
    
    // 连续测试数据流
    std::cout << "\nStarting continuous data stream test (10 seconds)..." << std::endl;
    auto start_time = std::chrono::steady_clock::now();
    int count = 0;
    
    while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(10)) {
        std::cout << "\n--- Data Stream " << ++count << " ---" << std::endl;
        
        tbot.getRobotDataStream(robotDataCallback);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    
    std::cout << "\nData stream test completed!" << std::endl;
    
    // 断开连接
    tbot.disconnect();
    std::cout << "Disconnected from TBot" << std::endl;
    
    return 0;
} 