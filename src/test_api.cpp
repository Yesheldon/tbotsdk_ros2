#include "tbot_sdk/TBotSDK.h"
#include <iostream>

void statusCallback(int code, const std::string& message) {
    std::cout << "Status: " << code << " - " << message << std::endl;
}

void mapDataCallback(const TBot::MapInfo& mapInfo) {
    std::cout << "Received map data. Resolution: " << mapInfo.resolution 
              << ", Size: " << mapInfo.width << "x" << mapInfo.height << std::endl;
}

void robotDataCallback(const TBot::RobotData& robotData) {
    std::cout << "Robot status: " << robotData.system_status 
              << ", Battery: " << robotData.battery_power << "%" << std::endl;
}

int main() {
    // 创建SDK实例
    TBot::TBotSDK tbot;
    
    // 连接底盘
    if (!tbot.connect(statusCallback)) {
        std::cerr << "Failed to connect to TBot" << std::endl;
        return 1;
    }
    
    // 检查服务状态
    tbot.checkServiceRunning(statusCallback);
    
    // 获取地图列表
    auto mapList = tbot.getMapList(statusCallback);
    std::cout << "Available maps:" << std::endl;
    for (const auto& mapName : mapList) {
        std::cout << " - " << mapName << std::endl;
    }
    
    // 启动导航服务
    if (!mapList.empty()) {
        tbot.startNavigation(mapList[0], statusCallback);
    }
    
    // 设置导航精度
    tbot.setNavigationPrecision(0.1f, 0.1f, statusCallback);
    
    // 导航到指定位置
    tbot.navigateTo(5.175f, -5.138f, -0.714f, 0.7f, statusCallback);
    
    // 获取机器人数据流
    tbot.getRobotDataStream(robotDataCallback);
    
    // 获取地图数据流
    tbot.getMapStream(mapDataCallback);
    
    // 回桩充电
    tbot.dockToCharger(0.18f, 0.0f, 0.0f, 1.0f, statusCallback);
    
    // 断开连接
    tbot.disconnect();
    
    return 0;
}