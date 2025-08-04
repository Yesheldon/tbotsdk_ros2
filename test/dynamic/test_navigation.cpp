#include <gtest/gtest.h>
#include "tbot_sdk/TBotSDK.h"
#include <memory>
#include <string>
#include <chrono>
#include <thread>

using namespace TBot;

class RealNavigationTest : public ::testing::Test {
protected:
    void SetUp() override {
        tbot_ = std::make_unique<TBotSDK>("192.168.8.110");
        tbot_->connect();
        
        // 确保机器人已定位
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
    
    void TearDown() override {
        // 停止所有导航
        if (tbot_->isConnected()) {
            tbot_->stopNavigation();
            tbot_->disconnect();
        }
        tbot_.reset();
    }
    
    std::unique_ptr<TBotSDK> tbot_;
};

// 测试启动导航
TEST_F(RealNavigationTest, TestStartNavigation) {
    auto maps = tbot_->getMapList();
    if (!maps.empty()) {
        bool result = tbot_->startNavigation(maps[0]);
        EXPECT_TRUE(result);
        
        // 验证导航已启动
        EXPECT_TRUE(tbot_->isNavigating());
        
        // 停止导航
        tbot_->stopNavigation();
        std::this_thread::sleep_for(std::chrono::seconds(1));
        EXPECT_FALSE(tbot_->isNavigating());
    }
}

// 测试导航到指定位置
TEST_F(RealNavigationTest, TestNavigateTo) {
    // 导航到当前位置附近的安全位置
    bool result = tbot_->navigateTo(1.0f, 1.0f, 0.0f, 0.5f);
    EXPECT_TRUE(result);
    
    // 等待导航开始
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    // 验证导航状态
    if (tbot_->isNavigating()) {
        std::cout << "Navigation is in progress..." << std::endl;
        
        // 等待一段时间让导航进行
        std::this_thread::sleep_for(std::chrono::seconds(5));
        
        // 停止导航
        tbot_->stopNavigation();
        std::this_thread::sleep_for(std::chrono::seconds(1));
        EXPECT_FALSE(tbot_->isNavigating());
    }
}

// 测试同步导航（短距离）
TEST_F(RealNavigationTest, TestNavigateToAndWait) {
    // 导航到当前位置附近的安全位置，设置较短的超时时间
    bool result = tbot_->navigateToAndWait(0.5f, 0.5f, 0.0f, 0.5f, 10);
    EXPECT_TRUE(result);
    
    std::cout << "Navigation completed or timed out" << std::endl;
}

// 测试停止导航
TEST_F(RealNavigationTest, TestStopNavigation) {
    // 先启动导航
    auto maps = tbot_->getMapList();
    if (!maps.empty()) {
        tbot_->startNavigation(maps[0]);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        // 验证导航已启动
        if (tbot_->isNavigating()) {
            // 停止导航
            bool result = tbot_->stopNavigation();
            EXPECT_TRUE(result);
            
            // 等待停止生效
            std::this_thread::sleep_for(std::chrono::seconds(1));
            EXPECT_FALSE(tbot_->isNavigating());
        }
    }
}

// 测试暂停和恢复导航
TEST_F(RealNavigationTest, TestPauseResumeNavigation) {
    auto maps = tbot_->getMapList();
    if (!maps.empty()) {
        // 启动导航
        tbot_->startNavigation(maps[0]);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        if (tbot_->isNavigating()) {
            // 暂停导航
            bool pause_result = tbot_->pauseNavigation();
            EXPECT_TRUE(pause_result);
            
            std::this_thread::sleep_for(std::chrono::seconds(2));
            
            // 恢复导航
            bool resume_result = tbot_->resumeNavigation();
            EXPECT_TRUE(resume_result);
            
            std::this_thread::sleep_for(std::chrono::seconds(2));
            
            // 停止导航
            tbot_->stopNavigation();
        }
    }
}

// 测试导航状态检查
TEST_F(RealNavigationTest, TestIsNavigating) {
    // 初始状态应该不是导航中
    EXPECT_FALSE(tbot_->isNavigating());
    
    // 启动导航
    auto maps = tbot_->getMapList();
    if (!maps.empty()) {
        tbot_->startNavigation(maps[0]);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        // 检查导航状态
        bool is_navigating = tbot_->isNavigating();
        std::cout << "Navigation status: " << (is_navigating ? "true" : "false") << std::endl;
        
        // 停止导航
        tbot_->stopNavigation();
        std::this_thread::sleep_for(std::chrono::seconds(1));
        EXPECT_FALSE(tbot_->isNavigating());
    }
}

// 测试导航参数验证
TEST_F(RealNavigationTest, TestNavigationParameterValidation) {
    // 测试无效坐标
    bool result = tbot_->navigateTo(999999.0f, 999999.0f, 0.0f, 0.0f);
    EXPECT_FALSE(result);
    
    // 测试无效地图
    bool start_result = tbot_->startNavigation("non_existent_map");
    EXPECT_FALSE(start_result);
}

// 测试导航精度设置
TEST_F(RealNavigationTest, TestNavigationPrecision) {
    // 设置导航精度
    bool result = tbot_->setNavigationPrecision(0.1f, 0.05f);
    EXPECT_TRUE(result);
    
    // 使用设置的精度进行导航
    bool nav_result = tbot_->navigateTo(1.0f, 1.0f, 0.0f, 0.5f);
    EXPECT_TRUE(nav_result);
    
    // 等待导航开始
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    if (tbot_->isNavigating()) {
        // 停止导航
        tbot_->stopNavigation();
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

// 测试导航状态转换
TEST_F(RealNavigationTest, TestNavigationStateTransitions) {
    auto maps = tbot_->getMapList();
    if (!maps.empty()) {
        // 状态1：启动导航
        bool started = tbot_->startNavigation(maps[0]);
        EXPECT_TRUE(started);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        if (tbot_->isNavigating()) {
            // 状态2：暂停导航
            bool paused = tbot_->pauseNavigation();
            EXPECT_TRUE(paused);
            std::this_thread::sleep_for(std::chrono::seconds(1));
            
            // 状态3：恢复导航
            bool resumed = tbot_->resumeNavigation();
            EXPECT_TRUE(resumed);
            std::this_thread::sleep_for(std::chrono::seconds(1));
            
            // 状态4：停止导航
            bool stopped = tbot_->stopNavigation();
            EXPECT_TRUE(stopped);
            std::this_thread::sleep_for(std::chrono::seconds(1));
            
            EXPECT_FALSE(tbot_->isNavigating());
        }
    }
}

// 测试导航到充电桩
TEST_F(RealNavigationTest, TestDockToCharger) {
    // 导航到充电桩
    bool result = tbot_->dockToCharger(0.0f, 0.0f, 0.0f, 1.0f);
    EXPECT_TRUE(result);
    
    // 等待导航开始
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    if (tbot_->isNavigating()) {
        std::cout << "Docking to charger..." << std::endl;
        
        // 等待一段时间让导航进行
        std::this_thread::sleep_for(std::chrono::seconds(10));
        
        // 停止导航
        tbot_->stopNavigation();
    }
}

// 测试重新定位
TEST_F(RealNavigationTest, TestRelocate) {
    // 重新定位机器人
    bool result = tbot_->relocate(0.0f, 0.0f, 0.0f, 1.0f);
    EXPECT_TRUE(result);
    
    std::cout << "Relocation completed" << std::endl;
}

// 测试导航错误处理
TEST_F(RealNavigationTest, TestNavigationErrorHandling) {
    // 测试在未连接状态下导航
    auto disconnected_tbot = std::make_unique<TBotSDK>("192.168.8.110");
    
    bool result = disconnected_tbot->navigateTo(1.0f, 1.0f, 0.0f, 0.5f);
    EXPECT_FALSE(result);
    
    bool start_result = disconnected_tbot->startNavigation("test_map");
    EXPECT_FALSE(start_result);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
} 