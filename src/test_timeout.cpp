#include "tbot_sdk/TBotSDK.h"
#include <iostream>
#include <chrono>
#include <thread>

using namespace TBot;

void testTimeout() {
    TBotSDK tbot;
    
    // 测试连接超时
    std::cout << "=== 测试连接超时 ===" << std::endl;
    auto start = std::chrono::steady_clock::now();
    
    bool connected = tbot.connect([](int code, const std::string& msg) {
        std::cout << "连接结果: " << code << " - " << msg << std::endl;
    });
    
    auto end = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(end - start);
    std::cout << "连接耗时: " << duration.count() << " 秒" << std::endl;
    
    if (!connected) {
        std::cout << "连接失败，跳过后续测试" << std::endl;
        return;
    }
    
    // 测试建图超时
    std::cout << "\n=== 测试建图超时 ===" << std::endl;
    start = std::chrono::steady_clock::now();
    
    bool mappingStarted = tbot.startMapping(5, [](int code, const std::string& msg) {
        std::cout << "建图结果: " << code << " - " << msg << std::endl;
    });
    
    end = std::chrono::steady_clock::now();
    duration = std::chrono::duration_cast<std::chrono::seconds>(end - start);
    std::cout << "建图耗时: " << duration.count() << " 秒" << std::endl;
    
    // 测试数据流超时
    std::cout << "\n=== 测试数据流超时 ===" << std::endl;
    start = std::chrono::steady_clock::now();
    
    bool dataReceived = tbot.getRobotDataStream([](const RobotData& data) {
        std::cout << "收到机器人数据: " << data.system_status << std::endl;
    });
    
    end = std::chrono::steady_clock::now();
    duration = std::chrono::duration_cast<std::chrono::seconds>(end - start);
    std::cout << "数据流耗时: " << duration.count() << " 秒" << std::endl;
    
    // 测试导航超时
    std::cout << "\n=== 测试导航超时 ===" << std::endl;
    start = std::chrono::steady_clock::now();
    
    bool navigationStarted = tbot.navigateTo(1.0f, 1.0f, 0.0f, 1.0f, [](int code, const std::string& msg) {
        std::cout << "导航结果: " << code << " - " << msg << std::endl;
    });
    
    end = std::chrono::steady_clock::now();
    duration = std::chrono::duration_cast<std::chrono::seconds>(end - start);
    std::cout << "导航耗时: " << duration.count() << " 秒" << std::endl;
    
    // 测试同步导航超时
    std::cout << "\n=== 测试同步导航超时 ===" << std::endl;
    start = std::chrono::steady_clock::now();
    
    bool navigationCompleted = tbot.navigateToAndWait(2.0f, 2.0f, 0.0f, 1.0f, 30, [](int code, const std::string& msg) {
        std::cout << "同步导航结果: " << code << " - " << msg << std::endl;
    });
    
    end = std::chrono::steady_clock::now();
    duration = std::chrono::duration_cast<std::chrono::seconds>(end - start);
    std::cout << "同步导航耗时: " << duration.count() << " 秒" << std::endl;
}

int main() {
    std::cout << "开始测试超时机制..." << std::endl;
    testTimeout();
    std::cout << "测试完成" << std::endl;
    return 0;
} 