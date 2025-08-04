#include <gtest/gtest.h>
#include "tbot_sdk/TBotSDK.h"
#include <memory>
#include <string>
#include <vector>

using namespace TBot;

class RealMapStatusTest : public ::testing::Test {
protected:
    void SetUp() override {
        tbot_ = std::make_unique<TBotSDK>("192.168.8.110");
        tbot_->connect();
    }
    
    void TearDown() override {
        if (tbot_->isConnected()) {
            tbot_->disconnect();
        }
        tbot_.reset();
    }
    
    std::unique_ptr<TBotSDK> tbot_;
};

// 测试获取地图列表
TEST_F(RealMapStatusTest, TestGetMapList) {
    auto maps = tbot_->getMapList();
    
    // 验证地图列表不为空（假设机器人上至少有一个地图）
    EXPECT_FALSE(maps.empty());
    
    // 验证每个地图名称都是有效的
    for (const auto& map_name : maps) {
        EXPECT_FALSE(map_name.empty());
        EXPECT_GT(map_name.length(), 0);
        EXPECT_LT(map_name.length(), 100); // 地图名称不应该过长
    }
    
    std::cout << "Found " << maps.size() << " maps: ";
    for (const auto& map : maps) {
        std::cout << map << " ";
    }
    std::cout << std::endl;
}

// 测试获取当前地图
TEST_F(RealMapStatusTest, TestGetCurrentMap) {
    bool callback_called = false;
    
    bool result = tbot_->getCurrentMap([&callback_called](const MapInfo& mapInfo) {
        callback_called = true;
        
        // 验证地图信息的基本字段
        EXPECT_FALSE(mapInfo.name.empty());
        EXPECT_GT(mapInfo.resolution, 0);
        EXPECT_GT(mapInfo.width, 0);
        EXPECT_GT(mapInfo.height, 0);
        EXPECT_FALSE(mapInfo.data.empty());
        
        // 验证地图原点
        EXPECT_FINITE(mapInfo.origin.position.x);
        EXPECT_FINITE(mapInfo.origin.position.y);
        EXPECT_FINITE(mapInfo.origin.position.z);
        
        std::cout << "Current map: " << mapInfo.name 
                  << ", resolution: " << mapInfo.resolution
                  << ", size: " << mapInfo.width << "x" << mapInfo.height << std::endl;
    });
    
    EXPECT_TRUE(result);
    EXPECT_TRUE(callback_called);
}

// 测试地图是否已加载
TEST_F(RealMapStatusTest, TestIsMapLoaded) {
    bool is_loaded = tbot_->isMapLoaded();
    
    // 验证返回值为布尔值
    // 注意：这个值取决于机器人当前的状态
    std::cout << "Map loaded status: " << (is_loaded ? "true" : "false") << std::endl;
}

// 测试地图信息内容验证
TEST_F(RealMapStatusTest, TestMapInfoContent) {
    bool map_received = false;
    
    tbot_->getCurrentMap([&map_received](const MapInfo& mapInfo) {
        map_received = true;
        
        // 验证地图名称
        EXPECT_FALSE(mapInfo.name.empty());
        EXPECT_GT(mapInfo.name.length(), 0);
        
        // 验证地图分辨率
        EXPECT_GT(mapInfo.resolution, 0);
        EXPECT_LT(mapInfo.resolution, 1.0); // 分辨率应该小于1米
        
        // 验证地图尺寸
        EXPECT_GT(mapInfo.width, 0);
        EXPECT_GT(mapInfo.height, 0);
        EXPECT_LE(mapInfo.width, 10000); // 地图宽度不应该过大
        EXPECT_LE(mapInfo.height, 10000); // 地图高度不应该过大
        
        // 验证地图数据
        EXPECT_FALSE(mapInfo.data.empty());
        EXPECT_EQ(mapInfo.data.size(), mapInfo.width * mapInfo.height);
        
        // 验证地图原点
        EXPECT_FINITE(mapInfo.origin.position.x);
        EXPECT_FINITE(mapInfo.origin.position.y);
        EXPECT_FINITE(mapInfo.origin.position.z);
        EXPECT_FINITE(mapInfo.origin.orientation.x);
        EXPECT_FINITE(mapInfo.origin.orientation.y);
        EXPECT_FINITE(mapInfo.origin.orientation.z);
        EXPECT_FINITE(mapInfo.origin.orientation.w);
        
        // 验证地图数据内容
        bool has_occupied = false;
        bool has_free = false;
        bool has_unknown = false;
        
        for (int8_t cell : mapInfo.data) {
            if (cell > 0) has_occupied = true;
            else if (cell == 0) has_free = true;
            else has_unknown = true;
        }
        
        // 地图应该包含至少一些有效数据
        EXPECT_TRUE(has_free || has_occupied);
    });
    
    EXPECT_TRUE(map_received);
}

// 测试地图边界条件
TEST_F(RealMapStatusTest, TestMapBoundaryConditions) {
    auto maps = tbot_->getMapList();
    
    if (!maps.empty()) {
        // 测试切换到第一个地图
        bool switch_result = tbot_->switchMap(maps[0]);
        EXPECT_TRUE(switch_result);
        
        // 验证地图已加载
        bool is_loaded = tbot_->isMapLoaded();
        EXPECT_TRUE(is_loaded);
        
        // 获取切换后的地图信息
        bool map_received = false;
        tbot_->getCurrentMap([&map_received, &maps](const MapInfo& mapInfo) {
            map_received = true;
            EXPECT_EQ(mapInfo.name, maps[0]);
        });
        
        EXPECT_TRUE(map_received);
    }
}

// 测试地图分辨率验证
TEST_F(RealMapStatusTest, TestMapResolutionValidation) {
    tbot_->getCurrentMap([](const MapInfo& mapInfo) {
        // 验证分辨率在合理范围内
        EXPECT_GT(mapInfo.resolution, 0.01); // 分辨率应该大于1cm
        EXPECT_LT(mapInfo.resolution, 1.0);  // 分辨率应该小于1m
        
        // 分辨率应该是合理的值（常见值：0.05, 0.1, 0.2等）
        float resolution = mapInfo.resolution;
        bool is_reasonable = (resolution == 0.05f || resolution == 0.1f || 
                             resolution == 0.2f || resolution == 0.25f);
        
        if (!is_reasonable) {
            std::cout << "Warning: Unusual map resolution: " << resolution << std::endl;
        }
    });
}

// 测试地图尺寸验证
TEST_F(RealMapStatusTest, TestMapSizeValidation) {
    tbot_->getCurrentMap([](const MapInfo& mapInfo) {
        // 验证地图尺寸在合理范围内
        EXPECT_GT(mapInfo.width, 100);   // 地图宽度应该大于100像素
        EXPECT_GT(mapInfo.height, 100);  // 地图高度应该大于100像素
        EXPECT_LE(mapInfo.width, 10000); // 地图宽度不应该过大
        EXPECT_LE(mapInfo.height, 10000); // 地图高度不应该过大
        
        // 验证地图尺寸比例合理（不应该过于细长）
        float aspect_ratio = static_cast<float>(mapInfo.width) / mapInfo.height;
        EXPECT_GT(aspect_ratio, 0.1);  // 宽高比应该大于0.1
        EXPECT_LT(aspect_ratio, 10.0); // 宽高比应该小于10
        
        std::cout << "Map size: " << mapInfo.width << "x" << mapInfo.height 
                  << ", aspect ratio: " << aspect_ratio << std::endl;
    });
}

// 测试地图操作错误处理
TEST_F(RealMapStatusTest, TestMapOperationErrors) {
    // 测试切换到不存在的地图
    bool switch_result = tbot_->switchMap("non_existent_map");
    EXPECT_FALSE(switch_result);
    
    // 测试删除不存在的地图
    bool delete_result = tbot_->deleteMap("non_existent_map");
    EXPECT_FALSE(delete_result);
}

// 测试地图数据完整性
TEST_F(RealMapStatusTest, TestMapDataIntegrity) {
    tbot_->getCurrentMap([](const MapInfo& mapInfo) {
        // 验证地图数据大小正确
        size_t expected_size = mapInfo.width * mapInfo.height;
        EXPECT_EQ(mapInfo.data.size(), expected_size);
        
        // 验证地图数据内容
        int occupied_cells = 0;
        int free_cells = 0;
        int unknown_cells = 0;
        
        for (int8_t cell : mapInfo.data) {
            if (cell > 0) occupied_cells++;
            else if (cell == 0) free_cells++;
            else unknown_cells++;
        }
        
        // 地图应该包含一些有效数据
        EXPECT_GT(free_cells + occupied_cells, 0);
        
        std::cout << "Map data: " << occupied_cells << " occupied, " 
                  << free_cells << " free, " << unknown_cells << " unknown cells" << std::endl;
    });
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
} 