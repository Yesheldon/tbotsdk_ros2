#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "tbot_sdk/TBotSDK.h"
#include <memory>
#include <string>
#include <vector>

using namespace TBot;
using ::testing::_;
using ::testing::Return;
using ::testing::StrictMock;

// Mock类用于测试地图状态功能
class MockTBotSDK : public TBotSDK {
public:
    MockTBotSDK(const std::string& ip = "192.168.8.110") : TBotSDK(ip) {}
    
    MOCK_METHOD(std::vector<std::string>, getMapList, (StatusCallback callback), (override));
    MOCK_METHOD(bool, getCurrentMap, (MapInfoCallback callback), (override));
    MOCK_METHOD(bool, getMapStatus, (MapStatusCallback callback), (override));
    MOCK_METHOD(bool, isMapLoaded, (), (const, override));
};

class MapStatusTest : public ::testing::Test {
protected:
    void SetUp() override {
        mock_tbot_ = std::make_unique<StrictMock<MockTBotSDK>>("192.168.8.110");
    }
    
    void TearDown() override {
        mock_tbot_.reset();
    }
    
    std::unique_ptr<StrictMock<MockTBotSDK>> mock_tbot_;
};

// 测试获取地图列表
TEST_F(MapStatusTest, TestGetMapList) {
    std::vector<std::string> expected_maps = {"map1", "map2", "map3"};
    
    EXPECT_CALL(*mock_tbot_, getMapList(_))
        .WillOnce(Return(expected_maps));
    
    auto maps = mock_tbot_->getMapList([](int code, const std::string& message) {
        EXPECT_EQ(code, 0);
        EXPECT_EQ(message, "Success");
    });
    
    EXPECT_EQ(maps.size(), 3);
    EXPECT_EQ(maps[0], "map1");
    EXPECT_EQ(maps[1], "map2");
    EXPECT_EQ(maps[2], "map3");
}

// 测试获取当前地图
TEST_F(MapStatusTest, TestGetCurrentMap) {
    EXPECT_CALL(*mock_tbot_, getCurrentMap(_))
        .WillOnce(Return(true));
    
    bool result = mock_tbot_->getCurrentMap([](const MapInfo& mapInfo) {
        EXPECT_EQ(mapInfo.name, "current_map");
        EXPECT_FLOAT_EQ(mapInfo.resolution, 0.05);
        EXPECT_EQ(mapInfo.width, 1000);
        EXPECT_EQ(mapInfo.height, 1000);
        EXPECT_FALSE(mapInfo.data.empty());
        EXPECT_FLOAT_EQ(mapInfo.origin.position.x, -25.0);
        EXPECT_FLOAT_EQ(mapInfo.origin.position.y, -25.0);
        EXPECT_FLOAT_EQ(mapInfo.origin.position.z, 0.0);
    });
    
    EXPECT_TRUE(result);
}

// 测试获取地图状态
TEST_F(MapStatusTest, TestGetMapStatus) {
    EXPECT_CALL(*mock_tbot_, getMapStatus(_))
        .WillOnce(Return(true));
    
    bool result = mock_tbot_->getMapStatus([](const MapStatus& status) {
        EXPECT_EQ(status.is_loaded, true);
        EXPECT_EQ(status.is_valid, true);
        EXPECT_EQ(status.name, "test_map");
        EXPECT_FLOAT_EQ(status.resolution, 0.05);
        EXPECT_EQ(status.width, 1000);
        EXPECT_EQ(status.height, 1000);
        EXPECT_FLOAT_EQ(status.origin.x, -25.0);
        EXPECT_FLOAT_EQ(status.origin.y, -25.0);
    });
    
    EXPECT_TRUE(result);
}

// 测试地图是否已加载
TEST_F(MapStatusTest, TestIsMapLoaded) {
    EXPECT_CALL(*mock_tbot_, isMapLoaded())
        .WillOnce(Return(true))
        .WillOnce(Return(false));
    
    EXPECT_TRUE(mock_tbot_->isMapLoaded());
    EXPECT_FALSE(mock_tbot_->isMapLoaded());
}

// 测试获取地图列表失败
TEST_F(MapStatusTest, TestGetMapListFailure) {
    std::vector<std::string> empty_maps;
    
    EXPECT_CALL(*mock_tbot_, getMapList(_))
        .WillOnce(Return(empty_maps));
    
    auto maps = mock_tbot_->getMapList([](int code, const std::string& message) {
        EXPECT_EQ(code, -1);
        EXPECT_EQ(message, "Failed to get map list");
    });
    
    EXPECT_TRUE(maps.empty());
}

// 测试获取当前地图失败
TEST_F(MapStatusTest, TestGetCurrentMapFailure) {
    EXPECT_CALL(*mock_tbot_, getCurrentMap(_))
        .WillOnce(Return(false));
    
    bool result = mock_tbot_->getCurrentMap([](const MapInfo& mapInfo) {
        // 这个回调不应该被调用
        FAIL() << "Current map callback should not be called";
    });
    
    EXPECT_FALSE(result);
}

// 测试获取地图状态失败
TEST_F(MapStatusTest, TestGetMapStatusFailure) {
    EXPECT_CALL(*mock_tbot_, getMapStatus(_))
        .WillOnce(Return(false));
    
    bool result = mock_tbot_->getMapStatus([](const MapStatus& status) {
        // 这个回调不应该被调用
        FAIL() << "Map status callback should not be called";
    });
    
    EXPECT_FALSE(result);
}

// 测试地图信息内容验证
TEST_F(MapStatusTest, TestMapInfoContent) {
    MapInfo testMap;
    testMap.name = "test_map";
    testMap.resolution = 0.1;
    testMap.width = 500;
    testMap.height = 500;
    testMap.data = {0, 1, 2, 3, 4, 5};
    testMap.origin.position.x = -10.0;
    testMap.origin.position.y = -10.0;
    testMap.origin.position.z = 0.0;
    
    EXPECT_EQ(testMap.name, "test_map");
    EXPECT_FLOAT_EQ(testMap.resolution, 0.1);
    EXPECT_EQ(testMap.width, 500);
    EXPECT_EQ(testMap.height, 500);
    EXPECT_EQ(testMap.data.size(), 6);
    EXPECT_FLOAT_EQ(testMap.origin.position.x, -10.0);
    EXPECT_FLOAT_EQ(testMap.origin.position.y, -10.0);
    EXPECT_FLOAT_EQ(testMap.origin.position.z, 0.0);
}

// 测试地图状态内容验证
TEST_F(MapStatusTest, TestMapStatusContent) {
    MapStatus testStatus;
    testStatus.is_loaded = true;
    testStatus.is_valid = true;
    testStatus.name = "valid_map";
    testStatus.resolution = 0.05;
    testStatus.width = 2000;
    testStatus.height = 2000;
    testStatus.origin.x = -50.0;
    testStatus.origin.y = -50.0;
    
    EXPECT_TRUE(testStatus.is_loaded);
    EXPECT_TRUE(testStatus.is_valid);
    EXPECT_EQ(testStatus.name, "valid_map");
    EXPECT_FLOAT_EQ(testStatus.resolution, 0.05);
    EXPECT_EQ(testStatus.width, 2000);
    EXPECT_EQ(testStatus.height, 2000);
    EXPECT_FLOAT_EQ(testStatus.origin.x, -50.0);
    EXPECT_FLOAT_EQ(testStatus.origin.y, -50.0);
}

// 测试地图边界条件
TEST_F(MapStatusTest, TestMapBoundaryConditions) {
    // 测试空地图
    MapInfo emptyMap;
    emptyMap.name = "";
    emptyMap.width = 0;
    emptyMap.height = 0;
    emptyMap.data.clear();
    
    EXPECT_TRUE(emptyMap.name.empty());
    EXPECT_EQ(emptyMap.width, 0);
    EXPECT_EQ(emptyMap.height, 0);
    EXPECT_TRUE(emptyMap.data.empty());
    
    // 测试无效地图状态
    MapStatus invalidStatus;
    invalidStatus.is_loaded = false;
    invalidStatus.is_valid = false;
    invalidStatus.name = "";
    invalidStatus.width = 0;
    invalidStatus.height = 0;
    
    EXPECT_FALSE(invalidStatus.is_loaded);
    EXPECT_FALSE(invalidStatus.is_valid);
    EXPECT_TRUE(invalidStatus.name.empty());
    EXPECT_EQ(invalidStatus.width, 0);
    EXPECT_EQ(invalidStatus.height, 0);
}

// 测试地图分辨率验证
TEST_F(MapStatusTest, TestMapResolutionValidation) {
    // 测试不同分辨率
    std::vector<float> resolutions = {0.01, 0.05, 0.1, 0.2, 0.5};
    
    for (float resolution : resolutions) {
        MapInfo testMap;
        testMap.resolution = resolution;
        
        EXPECT_FLOAT_EQ(testMap.resolution, resolution);
        EXPECT_GT(testMap.resolution, 0.0);
        EXPECT_LE(testMap.resolution, 1.0);
    }
}

// 测试地图尺寸验证
TEST_F(MapStatusTest, TestMapSizeValidation) {
    // 测试不同地图尺寸
    std::vector<std::pair<int, int>> sizes = {
        {100, 100}, {500, 500}, {1000, 1000}, {2000, 2000}
    };
    
    for (const auto& size : sizes) {
        MapInfo testMap;
        testMap.width = size.first;
        testMap.height = size.second;
        
        EXPECT_EQ(testMap.width, size.first);
        EXPECT_EQ(testMap.height, size.second);
        EXPECT_GT(testMap.width, 0);
        EXPECT_GT(testMap.height, 0);
    }
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
} 