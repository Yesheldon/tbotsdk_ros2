#ifndef TBOT_SDK_H
#define TBOT_SDK_H

#include <string>
#include <vector>
#include <map>
#include <functional>

namespace TBot {

// 基础数据结构
struct Position {
    float x;
    float y;
    float z;
};

struct Orientation {
    float x;
    float y;
    float z;
    float w;
};

struct Pose {
    Position position;
    Orientation orientation;
};

struct MapInfo {
    float resolution;
    int width;
    int height;
    Pose origin;
    std::vector<int8_t> data;
};

struct RobotData {
    // 连接状态
    bool imu_connected;
    bool wheel_odom_connected;
    bool lidar_connected;
    
    // 位姿信息
    Pose robot_pose;
    Pose laser_pose;
    
    // 电池信息
    int battery_power;
    int charging_mode;
    
    // 路径信息
    std::vector<std::vector<float>> path_poses;
    
    // 激光雷达数据
    float angle_min;
    float angle_max;
    float angle_increment;
    float range_min;
    float range_max;
    std::vector<float> ranges;
    
    // 系统状态
    std::string system_status;
    
    // 超声波数据
    std::vector<std::map<std::string, float>> sonar_data;
    
    // 其他信息
    float cpu_temp;
    std::vector<float> current_speed;
    std::vector<float> max_speed;
    int localization_quality;
    bool collision_monitor_state;
};

// 位置点信息结构体
struct Waypoint {
    std::string name;        // 位置点名称，如 "A点", "B点", "C点"
    Pose pose;               // 位置和姿态
    std::string description; // 位置点描述
    std::string map_name;    // 所属地图名称
    std::string created_time; // 创建时间
};

// 回调函数类型定义
using StatusCallback = std::function<void(int code, const std::string& message)>;
using MapDataCallback = std::function<void(const MapInfo& mapInfo)>;
using RobotDataCallback = std::function<void(const RobotData& robotData)>;
using WaypointCallback = std::function<void(const Waypoint& waypoint)>;
using WaypointListCallback = std::function<void(const std::vector<Waypoint>& waypoints)>;

class TBotSDK {
public:
    // 构造函数和析构函数
    TBotSDK(const std::string& ip = "192.168.8.110");
    ~TBotSDK();
    
    // 连接管理
    bool connect(StatusCallback callback = nullptr);
    void disconnect();
    bool isConnected() const;
    
    // 服务状态检查
    bool checkServiceRunning(StatusCallback callback = nullptr);
    
    // 建图服务
    bool startMapping(int resolution_cm = 5, StatusCallback callback = nullptr);
    bool stopMapping(StatusCallback callback = nullptr);
    bool saveMap(const std::string& mapName, StatusCallback callback = nullptr);
    
    // 导航服务
    bool startNavigation(const std::string& mapName, StatusCallback callback = nullptr);
    std::vector<std::string> getMapList(StatusCallback callback = nullptr);
    bool switchMap(const std::string& mapName, StatusCallback callback = nullptr);
    bool deleteMap(const std::string& mapName, StatusCallback callback = nullptr);
    
    // 地图数据获取
    bool getMapStream(MapDataCallback callback);
    bool getRobotDataStream(RobotDataCallback callback);
    
    // 导航控制
    bool setNavigationPrecision(float preciseXy, float preciseYaw, StatusCallback callback = nullptr);
    bool navigateTo(float x, float y, float z, float w, StatusCallback callback = nullptr);
    // 同步导航，结束后自动调用confirmStatus
    bool navigateToAndWait(float x, float y, float z, float w, int timeout_sec = 60, StatusCallback callback = nullptr);
    bool dockToCharger(float x, float y, float z, float w, StatusCallback callback = nullptr);
    bool resetDockerPose(float x, float y, float z, float w, StatusCallback callback = nullptr);
    bool stopMovement(StatusCallback callback = nullptr);
    bool relocate(float x, float y, float z, float w, StatusCallback callback = nullptr);
    bool confirmStatus(StatusCallback callback = nullptr);
    
    // 手动控制
    bool setMaxSpeed(float linearSpeed, float angularSpeed, StatusCallback callback = nullptr);
    bool velocityControl(float linearSpeed, float angularSpeed, StatusCallback callback = nullptr);
    bool move(const std::string& action, float dist, float speed, int timeAllowance, StatusCallback callback = nullptr);
    
    // 安全设置
    bool enableCollisionMonitor(bool enable, StatusCallback callback = nullptr);
    bool updateSafetyZones(const std::map<std::string, std::map<std::string, float>>& zones, StatusCallback callback = nullptr);
    
    // 系统信息
    std::map<std::string, std::string> getSystemInfo(StatusCallback callback = nullptr);
    bool setWiFiHotspot(const std::string& name, const std::string& password, StatusCallback callback = nullptr);
    bool connectToWiFi(const std::string& ssid, const std::string& password, StatusCallback callback = nullptr);
    
    // 数据持久化
    bool uploadCustomData(const std::string& name, const std::string& jsonData, StatusCallback callback = nullptr);
    std::string downloadCustomData(const std::string& name, StatusCallback callback = nullptr);
    bool deleteCustomData(const std::string& name, StatusCallback callback = nullptr);
    
    // 地图编辑
    bool getEditedMapData(const std::string& mapName, std::vector<int>& path, std::vector<int>& wall, StatusCallback callback = nullptr);
    bool saveEditedMap(const std::string& mapName, const std::vector<int>& path, const std::vector<int>& wall, StatusCallback callback = nullptr);
    
    // 位置点管理
    bool saveWaypoint(const std::string& name, const Pose& pose, const std::string& description = "", const std::string& mapName = "", StatusCallback callback = nullptr);
    bool saveCurrentPositionAsWaypoint(const std::string& name, const std::string& description = "", StatusCallback callback = nullptr);
    bool loadWaypoint(const std::string& name, WaypointCallback callback = nullptr);
    std::vector<Waypoint> getAllWaypoints(const std::string& mapName = "", StatusCallback callback = nullptr);
    bool deleteWaypoint(const std::string& name, StatusCallback callback = nullptr);
    bool navigateToWaypoint(const std::string& name, StatusCallback callback = nullptr);
    bool navigateToWaypointAndWait(const std::string& name, int timeout_sec = 60, StatusCallback callback = nullptr);
    
private:
    std::string m_ip;
    bool m_connected;
    // 其他私有成员和方法...
};

} // namespace TBot

#endif // TBOT_SDK_H