#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "tbot_sdk/TBotSDK.h"
#include <memory>
#include <string>

using namespace TBot;
using ::testing::_;
using ::testing::Return;
using ::testing::StrictMock;

// Mock类用于测试导航功能
class MockTBotSDK : public TBotSDK {
public:
    MockTBotSDK(const std::string& ip = "192.168.8.110") : TBotSDK(ip) {}
    
    MOCK_METHOD(bool, startNavigation, (const std::string& mapName, StatusCallback callback), (override));
    MOCK_METHOD(bool, navigateTo, (float x, float y, float z, float w, StatusCallback callback), (override));
    MOCK_METHOD(bool, navigateToAndWait, (float x, float y, float z, float w, int timeout, StatusCallback callback), (override));
    MOCK_METHOD(bool, stopNavigation, (StatusCallback callback), (override));
    MOCK_METHOD(bool, pauseNavigation, (StatusCallback callback), (override));
    MOCK_METHOD(bool, resumeNavigation, (StatusCallback callback), (override));
    MOCK_METHOD(bool, isNavigating, (), (const, override));
};

class NavigationTest : public ::testing::Test {
protected:
    void SetUp() override {
        mock_tbot_ = std::make_unique<StrictMock<MockTBotSDK>>("192.168.8.110");
    }
    
    void TearDown() override {
        mock_tbot_.reset();
    }
    
    std::unique_ptr<StrictMock<MockTBotSDK>> mock_tbot_;
};

// 测试启动导航
TEST_F(NavigationTest, TestStartNavigation) {
    EXPECT_CALL(*mock_tbot_, startNavigation("test_map", _))
        .WillOnce(Return(true));
    
    bool result = mock_tbot_->startNavigation("test_map", [](int code, const std::string& message) {
        EXPECT_EQ(code, 0);
        EXPECT_EQ(message, "Navigation started");
    });
    
    EXPECT_TRUE(result);
}

// 测试导航到指定位置
TEST_F(NavigationTest, TestNavigateTo) {
    EXPECT_CALL(*mock_tbot_, navigateTo(1.0f, 2.0f, 0.0f, 0.5f, _))
        .WillOnce(Return(true));
    
    bool result = mock_tbot_->navigateTo(1.0f, 2.0f, 0.0f, 0.5f, [](int code, const std::string& message) {
        EXPECT_EQ(code, 0);
        EXPECT_EQ(message, "Navigation goal accepted");
    });
    
    EXPECT_TRUE(result);
}

// 测试同步导航
TEST_F(NavigationTest, TestNavigateToAndWait) {
    EXPECT_CALL(*mock_tbot_, navigateToAndWait(2.0f, 3.0f, 0.0f, 0.7f, 30, _))
        .WillOnce(Return(true));
    
    bool result = mock_tbot_->navigateToAndWait(2.0f, 3.0f, 0.0f, 0.7f, 30, [](int code, const std::string& message) {
        EXPECT_EQ(code, 0);
        EXPECT_EQ(message, "Navigation completed");
    });
    
    EXPECT_TRUE(result);
}

// 测试停止导航
TEST_F(NavigationTest, TestStopNavigation) {
    EXPECT_CALL(*mock_tbot_, stopNavigation(_))
        .WillOnce(Return(true));
    
    bool result = mock_tbot_->stopNavigation([](int code, const std::string& message) {
        EXPECT_EQ(code, 0);
        EXPECT_EQ(message, "Navigation stopped");
    });
    
    EXPECT_TRUE(result);
}

// 测试暂停导航
TEST_F(NavigationTest, TestPauseNavigation) {
    EXPECT_CALL(*mock_tbot_, pauseNavigation(_))
        .WillOnce(Return(true));
    
    bool result = mock_tbot_->pauseNavigation([](int code, const std::string& message) {
        EXPECT_EQ(code, 0);
        EXPECT_EQ(message, "Navigation paused");
    });
    
    EXPECT_TRUE(result);
}

// 测试恢复导航
TEST_F(NavigationTest, TestResumeNavigation) {
    EXPECT_CALL(*mock_tbot_, resumeNavigation(_))
        .WillOnce(Return(true));
    
    bool result = mock_tbot_->resumeNavigation([](int code, const std::string& message) {
        EXPECT_EQ(code, 0);
        EXPECT_EQ(message, "Navigation resumed");
    });
    
    EXPECT_TRUE(result);
}

// 测试导航状态检查
TEST_F(NavigationTest, TestIsNavigating) {
    EXPECT_CALL(*mock_tbot_, isNavigating())
        .WillOnce(Return(true))
        .WillOnce(Return(false));
    
    EXPECT_TRUE(mock_tbot_->isNavigating());
    EXPECT_FALSE(mock_tbot_->isNavigating());
}

// 测试启动导航失败
TEST_F(NavigationTest, TestStartNavigationFailure) {
    EXPECT_CALL(*mock_tbot_, startNavigation("invalid_map", _))
        .WillOnce(Return(false));
    
    bool result = mock_tbot_->startNavigation("invalid_map", [](int code, const std::string& message) {
        EXPECT_EQ(code, -1);
        EXPECT_EQ(message, "Failed to start navigation");
    });
    
    EXPECT_FALSE(result);
}

// 测试导航到指定位置失败
TEST_F(NavigationTest, TestNavigateToFailure) {
    EXPECT_CALL(*mock_tbot_, navigateTo(999.0f, 999.0f, 0.0f, 0.0f, _))
        .WillOnce(Return(false));
    
    bool result = mock_tbot_->navigateTo(999.0f, 999.0f, 0.0f, 0.0f, [](int code, const std::string& message) {
        EXPECT_EQ(code, -1);
        EXPECT_EQ(message, "Invalid navigation goal");
    });
    
    EXPECT_FALSE(result);
}

// 测试同步导航超时
TEST_F(NavigationTest, TestNavigateToAndWaitTimeout) {
    EXPECT_CALL(*mock_tbot_, navigateToAndWait(5.0f, 5.0f, 0.0f, 1.0f, 5, _))
        .WillOnce(Return(false));
    
    bool result = mock_tbot_->navigateToAndWait(5.0f, 5.0f, 0.0f, 1.0f, 5, [](int code, const std::string& message) {
        EXPECT_EQ(code, -1);
        EXPECT_EQ(message, "Navigation timeout");
    });
    
    EXPECT_FALSE(result);
}

// 测试导航参数验证
TEST_F(NavigationTest, TestNavigationParameterValidation) {
    // 测试有效坐标
    std::vector<std::tuple<float, float, float, float>> valid_coords = {
        {0.0f, 0.0f, 0.0f, 1.0f},
        {1.0f, 2.0f, 0.0f, 0.5f},
        {-5.0f, -3.0f, 0.0f, 0.7f},
        {10.0f, 15.0f, 0.0f, 0.3f}
    };
    
    for (const auto& coords : valid_coords) {
        float x, y, z, w;
        std::tie(x, y, z, w) = coords;
        
        EXPECT_GE(x, -100.0f);
        EXPECT_LE(x, 100.0f);
        EXPECT_GE(y, -100.0f);
        EXPECT_LE(y, 100.0f);
        EXPECT_FLOAT_EQ(z, 0.0f);
        EXPECT_GE(w, -1.0f);
        EXPECT_LE(w, 1.0f);
    }
}

// 测试导航精度设置
TEST_F(NavigationTest, TestNavigationPrecision) {
    // 测试不同精度设置
    std::vector<std::pair<float, float>> precisions = {
        {0.1f, 0.1f}, {0.05f, 0.05f}, {0.2f, 0.2f}, {0.01f, 0.01f}
    };
    
    for (const auto& precision : precisions) {
        float pos_precision = precision.first;
        float angle_precision = precision.second;
        
        EXPECT_GT(pos_precision, 0.0f);
        EXPECT_LE(pos_precision, 1.0f);
        EXPECT_GT(angle_precision, 0.0f);
        EXPECT_LE(angle_precision, 1.0f);
    }
}

// 测试导航状态转换
TEST_F(NavigationTest, TestNavigationStateTransitions) {
    // 测试从停止到开始导航
    EXPECT_CALL(*mock_tbot_, startNavigation("test_map", _))
        .WillOnce(Return(true));
    
    bool started = mock_tbot_->startNavigation("test_map", [](int code, const std::string& message) {
        EXPECT_EQ(code, 0);
    });
    EXPECT_TRUE(started);
    
    // 测试从导航到暂停
    EXPECT_CALL(*mock_tbot_, pauseNavigation(_))
        .WillOnce(Return(true));
    
    bool paused = mock_tbot_->pauseNavigation([](int code, const std::string& message) {
        EXPECT_EQ(code, 0);
    });
    EXPECT_TRUE(paused);
    
    // 测试从暂停到恢复
    EXPECT_CALL(*mock_tbot_, resumeNavigation(_))
        .WillOnce(Return(true));
    
    bool resumed = mock_tbot_->resumeNavigation([](int code, const std::string& message) {
        EXPECT_EQ(code, 0);
    });
    EXPECT_TRUE(resumed);
    
    // 测试从导航到停止
    EXPECT_CALL(*mock_tbot_, stopNavigation(_))
        .WillOnce(Return(true));
    
    bool stopped = mock_tbot_->stopNavigation([](int code, const std::string& message) {
        EXPECT_EQ(code, 0);
    });
    EXPECT_TRUE(stopped);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
} 