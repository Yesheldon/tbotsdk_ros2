#include "tbot_sdk/TBotSDK.h"
#include <curl/curl.h>
#include <json/json.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <sstream>
#include <iostream>
#include <zlib.h>
#include <cstring>
#include <chrono> // Added for navigateToAndWait
#include <iomanip> // Added for time formatting
#include <sstream> // Added for string stream

namespace TBot {

// 辅助函数
namespace {
    // CURL写回调函数
    size_t writeCallback(void* contents, size_t size, size_t nmemb, std::string* output) {
        size_t totalSize = size * nmemb;
        output->append((char*)contents, totalSize);
        return totalSize;
    }

    // 发送GET请求（带超时）
    std::string sendGetRequest(const std::string& url, int timeout_sec = 30) {
        CURL* curl = curl_easy_init();
        std::string response;
        
        if (curl) {
            curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
            curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writeCallback);
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
            
            // 设置超时时间
            curl_easy_setopt(curl, CURLOPT_TIMEOUT, timeout_sec);
            curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 10); // 连接超时10秒
            
            CURLcode res = curl_easy_perform(curl);
            if (res != CURLE_OK) {
                std::cerr << "curl_easy_perform() failed: " << curl_easy_strerror(res) << std::endl;
                if (res == CURLE_OPERATION_TIMEDOUT) {
                    std::cerr << "Request timed out after " << timeout_sec << " seconds" << std::endl;
                }
            }
            
            curl_easy_cleanup(curl);
        }
        
        return response;
    }

    // 发送POST请求（带超时）
    std::string sendPostRequest(const std::string& url, const std::string& postData, int timeout_sec = 30) {
        CURL* curl = curl_easy_init();
        std::string response;
        
        if (curl) {
            curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
            curl_easy_setopt(curl, CURLOPT_POSTFIELDS, postData.c_str());
            curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writeCallback);
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
            
            // 设置超时时间
            curl_easy_setopt(curl, CURLOPT_TIMEOUT, timeout_sec);
            curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 10); // 连接超时10秒
            
            CURLcode res = curl_easy_perform(curl);
            if (res != CURLE_OK) {
                std::cerr << "curl_easy_perform() failed: " << curl_easy_strerror(res) << std::endl;
                if (res == CURLE_OPERATION_TIMEDOUT) {
                    std::cerr << "Request timed out after " << timeout_sec << " seconds" << std::endl;
                }
            }
            
            curl_easy_cleanup(curl);
        }
        
        return response;
    }

    // 解析JSON响应
    bool parseResponse(const std::string& response, Json::Value& root) {
        Json::CharReaderBuilder builder;
        Json::CharReader* reader = builder.newCharReader();
        std::string errors;
        
        bool parsingSuccessful = reader->parse(response.c_str(), response.c_str() + response.size(), &root, &errors);
        delete reader;
        
        if (!parsingSuccessful) {
            std::cerr << "Failed to parse JSON: " << errors << std::endl;
            return false;
        }
        
        return true;
    }

    // Gzip解压缩函数
    bool decompressGzip(const std::string& compressed, std::string& decompressed) {
        z_stream strm;
        strm.zalloc = Z_NULL;
        strm.zfree = Z_NULL;
        strm.opaque = Z_NULL;
        strm.avail_in = compressed.size();
        strm.next_in = (Bytef*)compressed.data();
        
        if (inflateInit2(&strm, 16 + MAX_WBITS) != Z_OK) {
            return false;
        }
        
        char buffer[4096];
        int ret;
        
        do {
            strm.avail_out = sizeof(buffer);
            strm.next_out = (Bytef*)buffer;
            ret = inflate(&strm, Z_NO_FLUSH);
            
            if (ret != Z_STREAM_ERROR) {
                decompressed.append(buffer, sizeof(buffer) - strm.avail_out);
            }
        } while (ret == Z_OK);
        
        inflateEnd(&strm);
        return ret == Z_STREAM_END;
    }

    // 解析机器人数据
    bool parseRobotData(const std::string& jsonData, RobotData& robotData) {
        Json::Value root;
        if (!parseResponse(jsonData, root)) {
            return false;
        }
        
        try {
            // 解析连接状态
            robotData.imu_connected = root["imu_connected"].asBool();
            robotData.wheel_odom_connected = root["wheel_odom_connected"].asBool();
            robotData.lidar_connected = root["lidar_connected"].asBool();
            
            // 解析位姿信息
            if (root.isMember("robot_pose")) {
                Json::Value pose = root["robot_pose"];
                robotData.robot_pose.position.x = pose["position"]["x"].asFloat();
                robotData.robot_pose.position.y = pose["position"]["y"].asFloat();
                robotData.robot_pose.position.z = pose["position"]["z"].asFloat();
                robotData.robot_pose.orientation.x = pose["orientation"]["x"].asFloat();
                robotData.robot_pose.orientation.y = pose["orientation"]["y"].asFloat();
                robotData.robot_pose.orientation.z = pose["orientation"]["z"].asFloat();
                robotData.robot_pose.orientation.w = pose["orientation"]["w"].asFloat();
            }
            
            // 解析电池信息
            robotData.battery_power = root["battery_power"].asInt();
            robotData.charging_mode = root["charging_mode"].asInt();
            
            // 解析激光雷达数据
            if (root.isMember("laser_data")) {
                Json::Value laser = root["laser_data"];
                robotData.angle_min = laser["angle_min"].asFloat();
                robotData.angle_max = laser["angle_max"].asFloat();
                robotData.angle_increment = laser["angle_increment"].asFloat();
                robotData.range_min = laser["range_min"].asFloat();
                robotData.range_max = laser["range_max"].asFloat();
                
                Json::Value ranges = laser["ranges"];
                robotData.ranges.clear();
                for (const auto& range : ranges) {
                    robotData.ranges.push_back(range.asFloat());
                }
            }
            
            // 解析系统状态
            robotData.system_status = root["system_status"].asString();
            
            // 解析其他信息
            robotData.cpu_temp = root["cpu_temp"].asFloat();
            robotData.localization_quality = root["localization_quality"].asInt();
            robotData.collision_monitor_state = root["collision_monitor_state"].asBool();
            
            return true;
        } catch (const std::exception& e) {
            std::cerr << "Error parsing robot data: " << e.what() << std::endl;
            return false;
        }
    }

    // 解析地图数据
    bool parseMapData(const std::string& jsonData, MapInfo& mapInfo) {
        Json::Value root;
        if (!parseResponse(jsonData, root)) {
            return false;
        }
        
        try {
            mapInfo.resolution = root["resolution"].asFloat();
            mapInfo.width = root["width"].asInt();
            mapInfo.height = root["height"].asInt();
            
            // 解析原点信息
            if (root.isMember("origin")) {
                Json::Value origin = root["origin"];
                mapInfo.origin.position.x = origin["position"]["x"].asFloat();
                mapInfo.origin.position.y = origin["position"]["y"].asFloat();
                mapInfo.origin.position.z = origin["position"]["z"].asFloat();
                mapInfo.origin.orientation.x = origin["orientation"]["x"].asFloat();
                mapInfo.origin.orientation.y = origin["orientation"]["y"].asFloat();
                mapInfo.origin.orientation.z = origin["orientation"]["z"].asFloat();
                mapInfo.origin.orientation.w = origin["orientation"]["w"].asFloat();
            }
            
            // 解析地图数据
            if (root.isMember("data")) {
                Json::Value data = root["data"];
                mapInfo.data.clear();
                for (const auto& value : data) {
                    mapInfo.data.push_back(value.asInt());
                }
            }
            
            return true;
        } catch (const std::exception& e) {
            std::cerr << "Error parsing map data: " << e.what() << std::endl;
            return false;
        }
    }
}

// TBotSDK实现
TBotSDK::TBotSDK(const std::string& ip) : m_ip(ip), m_connected(false) {
    curl_global_init(CURL_GLOBAL_ALL);
}

TBotSDK::~TBotSDK() {
    disconnect();
    curl_global_cleanup();
}

bool TBotSDK::connect(StatusCallback callback) {
    std::string url = "http://" + m_ip + ":5000/is_running";
    std::string response = sendGetRequest(url);
    
    Json::Value root;
    if (!parseResponse(response, root)) {
        if (callback) callback(-1, "Failed to parse response");
        return false;
    }
    
    int code = root["code"].asInt();
    if (code == 0 || code == 2) {
        m_connected = true;
        if (callback) callback(code, "Connected successfully");
        return true;
    }
    
    if (callback) callback(code, "Service not running");
    return false;
}

void TBotSDK::disconnect() {
    m_connected = false;
}

bool TBotSDK::isConnected() const {
    return m_connected;
}

bool TBotSDK::checkServiceRunning(StatusCallback callback) {
    std::string url = "http://" + m_ip + ":5000/is_running";
    std::string response = sendGetRequest(url);
    
    Json::Value root;
    if (!parseResponse(response, root)) {
        if (callback) callback(-1, "Failed to parse response");
        return false;
    }
    
    int code = root["code"].asInt();
    std::string message = root["message"].asString();
    
    if (callback) callback(code, message);
    return (code == 0 || code == 2);
}

bool TBotSDK::startMapping(int resolution_cm, StatusCallback callback) {
    std::string url = "http://" + m_ip + ":5000/mapping/" + std::to_string(resolution_cm) + "/diff";
    std::string response = sendGetRequest(url, 120); // 建图可能需要2分钟
    
    Json::Value root;
    if (!parseResponse(response, root)) {
        if (callback) callback(-1, "Failed to parse response");
        return false;
    }
    
    int code = root["code"].asInt();
    std::string msg = root["msg"].asString();
    
    if (callback) callback(code, msg);
    return (code == 0);
}

bool TBotSDK::stopMapping(StatusCallback callback) {
    std::string url = "http://" + m_ip + ":1234/pause_slam";
    std::string response = sendGetRequest(url);
    
    Json::Value root;
    if (!parseResponse(response, root)) {
        if (callback) callback(-1, "Failed to parse response");
        return false;
    }
    
    bool code = root["code"].asBool();
    
    if (callback) callback(code ? 0 : -1, code ? "Success" : "Failed");
    return code;
}

bool TBotSDK::saveMap(const std::string& mapName, StatusCallback callback) {
    std::string url = "http://" + m_ip + ":5000/save_map/" + mapName;
    std::string response = sendGetRequest(url, 60); // 保存地图可能需要1分钟
    
    Json::Value root;
    if (!parseResponse(response, root)) {
        if (callback) callback(-1, "Failed to parse response");
        return false;
    }
    
    int code = root["code"].asInt();
    
    if (callback) callback(code, code == 0 ? "Success" : "Failed");
    return (code == 0);
}

bool TBotSDK::startNavigation(const std::string& mapName, StatusCallback callback) {
    std::string url = "http://" + m_ip + ":5000/navigation/" + mapName + "/diff";
    std::string response = sendGetRequest(url, 60); // 启动导航可能需要1分钟
    
    Json::Value root;
    if (!parseResponse(response, root)) {
        if (callback) callback(-1, "Failed to parse response");
        return false;
    }
    
    int code = root["code"].asInt();
    std::string msg = root["msg"].asString();
    
    if (callback) callback(code, msg);
    return (code == 0);
}

std::vector<std::string> TBotSDK::getMapList(StatusCallback callback) {
    std::string url = "http://" + m_ip + ":5000/get_map_list";
    std::string response = sendGetRequest(url);
    
    Json::Value root;
    if (!parseResponse(response, root)) {
        if (callback) callback(-1, "Failed to parse response");
        return {};
    }
    
    std::vector<std::string> mapList;
    for (const auto& mapName : root) {
        mapList.push_back(mapName.asString());
    }
    
    if (callback) callback(0, "Success");
    return mapList;
}

bool TBotSDK::switchMap(const std::string& mapName, StatusCallback callback) {
    std::string url = "http://" + m_ip + ":1234/change_map/" + mapName;
    std::string response = sendGetRequest(url);
    
    Json::Value root;
    if (!parseResponse(response, root)) {
        if (callback) callback(-1, "Failed to parse response");
        return false;
    }
    
    int code = root["code"].asInt();
    
    if (callback) callback(code, code == 0 ? "Success" : "Failed");
    return (code == 0);
}

bool TBotSDK::deleteMap(const std::string& mapName, StatusCallback callback) {
    std::string url = "http://" + m_ip + ":5000/delete_map/" + mapName;
    std::string response = sendGetRequest(url);
    
    Json::Value root;
    if (!parseResponse(response, root)) {
        if (callback) callback(-1, "Failed to parse response");
        return false;
    }
    
    int code = root["code"].asInt();
    
    if (callback) callback(code, code == 0 ? "Success" : "Failed");
    return (code == 0);
}

bool TBotSDK::getMapStream(MapDataCallback callback) {
    if (!m_connected) {
        std::cerr << "Not connected to robot" << std::endl;
        return false;
    }
    
    std::string url = "http://" + m_ip + ":1234/map_stream";
    std::string response = sendGetRequest(url, 60); // 地图流可能需要更长时间
    
    if (response.empty()) {
        std::cerr << "Failed to get map stream data" << std::endl;
        return false;
    }
    
    // 尝试解压缩Gzip数据
    std::string decompressed;
    if (!decompressGzip(response, decompressed)) {
        // 如果不是Gzip压缩，直接使用原始数据
        decompressed = response;
    }
    
    // 解析JSON数据
    MapInfo mapInfo;
    if (!parseMapData(decompressed, mapInfo)) {
        std::cerr << "Failed to parse map data" << std::endl;
        return false;
    }
    
    if (callback) callback(mapInfo);
    return true;
}

bool TBotSDK::getRobotDataStream(RobotDataCallback callback) {
    if (!m_connected) {
        std::cerr << "Not connected to robot" << std::endl;
        return false;
    }
    
    std::string url = "http://" + m_ip + ":1234/robot_data.stream";
    std::string response = sendGetRequest(url, 60); // 机器人数据流可能需要更长时间
    
    if (response.empty()) {
        std::cerr << "Failed to get robot data stream" << std::endl;
        return false;
    }
    
    // 尝试解压缩Gzip数据
    std::string decompressed;
    if (!decompressGzip(response, decompressed)) {
        // 如果不是Gzip压缩，直接使用原始数据
        decompressed = response;
    }
    
    // 解析JSON数据
    RobotData robotData;
    if (!parseRobotData(decompressed, robotData)) {
        std::cerr << "Failed to parse robot data" << std::endl;
        return false;
    }
    
    if (callback) callback(robotData);
    return true;
}

bool TBotSDK::setNavigationPrecision(float preciseXy, float preciseYaw, StatusCallback callback) {
    std::ostringstream oss;
    oss.precision(3);
    oss << std::fixed << preciseXy << "/" << std::fixed << preciseYaw;
    
    std::string url = "http://" + m_ip + ":1234/set_precision/" + oss.str();
    std::string response = sendGetRequest(url);
    
    Json::Value root;
    if (!parseResponse(response, root)) {
        if (callback) callback(-1, "Failed to parse response");
        return false;
    }
    
    int code = root["code"].asInt();
    
    if (callback) callback(code, code == 0 ? "Success" : "Failed");
    return (code == 0);
}

bool TBotSDK::navigateTo(float x, float y, float z, float w, StatusCallback callback) {
    std::ostringstream oss;
    oss.precision(3);
    oss << std::fixed << x << "/" << std::fixed << y << "/" 
        << std::fixed << z << "/" << std::fixed << w;
    
    std::string url = "http://" + m_ip + ":1234/go_to/" + oss.str();
    std::string response = sendGetRequest(url);
    
    Json::Value root;
    if (!parseResponse(response, root)) {
        if (callback) callback(-1, "Failed to parse response");
        return false;
    }
    
    int code = root["code"].asInt();
    
    if (callback) callback(code, code == 0 ? "Success" : "Failed");
    return (code == 0);
}

bool TBotSDK::navigateToAndWait(float x, float y, float z, float w, int timeout_sec, StatusCallback callback) {
    if (!navigateTo(x, y, z, w, callback)) {
        if (callback) callback(-1, "Failed to send navigation command");
        return false;
    }
    auto start = std::chrono::steady_clock::now();
    std::string final_status;
    while (std::chrono::steady_clock::now() - start < std::chrono::seconds(timeout_sec)) {
        bool finished = false;
        bool success = false;
        getRobotDataStream([&](const RobotData& data) {
            if (data.system_status == "succeeded" || data.system_status == "failed" ||
                data.system_status == "canceled" || data.system_status == "charging") {
                final_status = data.system_status;
                finished = true;
                success = (data.system_status == "succeeded");
            }
        });
        if (finished) {
            // 必须调用confirm_status并等待
            if (!confirmStatus(callback)) {
                if (callback) callback(-1, "Confirm status failed");
                return false;
            }
            if (callback) callback(success ? 0 : -1, "Navigation finished: " + final_status);
            return success;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    if (callback) callback(-1, "Navigation timeout");
    return false;
}

bool TBotSDK::dockToCharger(float x, float y, float z, float w, StatusCallback callback) {
    std::ostringstream oss;
    oss.precision(3);
    oss << std::fixed << x << "/" << std::fixed << y << "/" 
        << std::fixed << z << "/" << std::fixed << w;
    
    std::string url = "http://" + m_ip + ":1234/docking_with_pose/" + oss.str();
    std::string response = sendGetRequest(url);
    
    Json::Value root;
    if (!parseResponse(response, root)) {
        if (callback) callback(-1, "Failed to parse response");
        return false;
    }
    
    int code = root["code"].asInt();
    
    if (callback) callback(code, code == 0 ? "Success" : "Failed");
    return (code == 0);
}

bool TBotSDK::resetDockerPose(float x, float y, float z, float w, StatusCallback callback) {
    std::ostringstream oss;
    oss.precision(3);
    oss << std::fixed << x << "/" << std::fixed << y << "/" 
        << std::fixed << z << "/" << std::fixed << w;
    
    std::string url = "http://" + m_ip + ":1234/set_docker_pose/" + oss.str();
    std::string response = sendGetRequest(url);
    
    Json::Value root;
    if (!parseResponse(response, root)) {
        if (callback) callback(-1, "Failed to parse response");
        return false;
    }
    
    int code = root["code"].asInt();
    
    if (callback) callback(code, code == 0 ? "Success" : "Failed");
    return (code == 0);
}

bool TBotSDK::stopMovement(StatusCallback callback) {
    std::string url = "http://" + m_ip + ":1234/cancel_task";
    std::string response = sendGetRequest(url);
    
    Json::Value root;
    if (!parseResponse(response, root)) {
        if (callback) callback(-1, "Failed to parse response");
        return false;
    }
    
    int code = root["code"].asInt();
    
    if (callback) callback(code, code == 0 ? "Success" : "Failed");
    return (code == 0);
}

bool TBotSDK::relocate(float x, float y, float z, float w, StatusCallback callback) {
    std::ostringstream oss;
    oss.precision(3);
    oss << std::fixed << x << "/" << std::fixed << y << "/" 
        << std::fixed << z << "/" << std::fixed << w;
    
    std::string url = "http://" + m_ip + ":1234/set_pose/" + oss.str();
    std::string response = sendGetRequest(url);
    
    Json::Value root;
    if (!parseResponse(response, root)) {
        if (callback) callback(-1, "Failed to parse response");
        return false;
    }
    
    int code = root["code"].asInt();
    
    if (callback) callback(code, code == 0 ? "Success" : "Failed");
    return (code == 0);
}

bool TBotSDK::confirmStatus(StatusCallback callback) {
    std::string url = "http://" + m_ip + ":1234/confirm_status";
    std::string response = sendGetRequest(url, 30); // confirm_status需要等待
    Json::Value root;
    if (!parseResponse(response, root)) {
        if (callback) callback(-1, "Failed to parse confirm_status response");
        return false;
    }
    int code = root["code"].asInt();
    if (callback) callback(code, code == 0 ? "Confirm status success" : "Confirm status failed");
    return code == 0;
}

bool TBotSDK::setMaxSpeed(float linearSpeed, float angularSpeed, StatusCallback callback) {
    std::ostringstream oss;
    oss.precision(3);
    oss << std::fixed << linearSpeed << "/" << std::fixed << angularSpeed;
    
    std::string url = "http://" + m_ip + ":1234/set_max_speed/" + oss.str();
    std::string response = sendGetRequest(url);
    
    Json::Value root;
    if (!parseResponse(response, root)) {
        if (callback) callback(-1, "Failed to parse response");
        return false;
    }
    
    int code = root["code"].asInt();
    
    if (callback) callback(code, code == 0 ? "Success" : "Failed");
    return (code == 0);
}

bool TBotSDK::velocityControl(float linearSpeed, float angularSpeed, StatusCallback callback) {
    std::ostringstream oss;
    oss.precision(3);
    oss << std::fixed << linearSpeed << "/" << std::fixed << angularSpeed;
    
    std::string url = "http://" + m_ip + ":1234/velocity_control/" + oss.str();
    std::string response = sendGetRequest(url);
    
    Json::Value root;
    if (!parseResponse(response, root)) {
        if (callback) callback(-1, "Failed to parse response");
        return false;
    }
    
    int code = root["code"].asInt();
    
    if (callback) callback(code, code == 0 ? "Success" : "Failed");
    return (code == 0);
}

bool TBotSDK::move(const std::string& action, float dist, float speed, int timeAllowance, StatusCallback callback) {
    Json::Value moveData;
    moveData["action"] = action;
    moveData["dist"] = dist;
    moveData["speed"] = speed;
    moveData["time_allowance"] = timeAllowance;
    
    Json::StreamWriterBuilder writer;
    std::string jsonData = Json::writeString(writer, moveData);
    
    std::string url = "http://" + m_ip + ":1234/move";
    std::string response = sendPostRequest(url, jsonData);
    
    Json::Value root;
    if (!parseResponse(response, root)) {
        if (callback) callback(-1, "Failed to parse response");
        return false;
    }
    
    int code = root["code"].asInt();
    
    if (callback) callback(code, code == 0 ? "Success" : "Failed");
    return (code == 0);
}

bool TBotSDK::enableCollisionMonitor(bool enable, StatusCallback callback) {
    std::string url = "http://" + m_ip + ":5000/parameter/enableCollisionMonitor/" + std::to_string(enable ? 1 : 0);
    std::string response = sendGetRequest(url);
    
    Json::Value root;
    if (!parseResponse(response, root)) {
        if (callback) callback(-1, "Failed to parse response");
        return false;
    }
    
    int code = root["code"].asInt();
    
    if (callback) callback(code, code == 0 ? "Success" : "Failed");
    return (code == 0);
}

bool TBotSDK::updateSafetyZones(const std::map<std::string, std::map<std::string, float>>& zones, StatusCallback callback) {
    Json::Value zonesData;
    for (const auto& zone : zones) {
        Json::Value zoneData;
        for (const auto& param : zone.second) {
            zoneData[param.first] = param.second;
        }
        zonesData[zone.first] = zoneData;
    }
    
    Json::StreamWriterBuilder writer;
    std::string jsonData = Json::writeString(writer, zonesData);
    
    std::string url = "http://" + m_ip + ":5000/parameter/updateCollisionMonitor";
    std::string response = sendPostRequest(url, jsonData);
    
    Json::Value root;
    if (!parseResponse(response, root)) {
        if (callback) callback(-1, "Failed to parse response");
        return false;
    }
    
    int code = root["code"].asInt();
    
    if (callback) callback(code, code == 0 ? "Success" : "Failed");
    return (code == 0);
}

std::map<std::string, std::string> TBotSDK::getSystemInfo(StatusCallback callback) {
    std::string url = "http://" + m_ip + ":5000/system/getSystemInfo";
    std::string response = sendGetRequest(url);
    
    Json::Value root;
    if (!parseResponse(response, root)) {
        if (callback) callback(-1, "Failed to parse response");
        return {};
    }
    
    std::map<std::string, std::string> systemInfo;
    
    if (root["code"].asInt() == 0) {
        Json::Value data = root["data"];
        systemInfo["ap_ssid"] = data["ap_ssid"].asString();
        systemInfo["cpu_temp"] = std::to_string(data["cpu_temp"].asFloat());
        systemInfo["cpu_usage"] = std::to_string(data["cpu_usage"].asFloat());
        systemInfo["wifi"] = data["wifi"].asString();
        
        Json::Value disk = data["disk_usage"];
        systemInfo["disk_size"] = disk["Size"].asString();
        systemInfo["disk_used"] = disk["Used"].asString();
        systemInfo["disk_available"] = disk["Available"].asString();
        systemInfo["disk_usage_percent"] = disk["Use%"].asString();
    }
    
    if (callback) callback(root["code"].asInt(), root["message"].asString());
    return systemInfo;
}

bool TBotSDK::setWiFiHotspot(const std::string& name, const std::string& password, StatusCallback callback) {
    Json::Value hotspotData;
    hotspotData["name"] = name;
    hotspotData["password_ap"] = password;
    
    Json::StreamWriterBuilder writer;
    std::string jsonData = Json::writeString(writer, hotspotData);
    
    std::string url = "http://" + m_ip + ":5000/system/setAPJson";
    std::string response = sendPostRequest(url, jsonData);
    
    Json::Value root;
    if (!parseResponse(response, root)) {
        if (callback) callback(-1, "Failed to parse response");
        return false;
    }
    
    int code = root["code"].asInt();
    std::string message = root["message"].asString();
    
    if (callback) callback(code, message);
    return (code == 0);
}

bool TBotSDK::connectToWiFi(const std::string& ssid, const std::string& password, StatusCallback callback) {
    Json::Value wifiData;
    wifiData["ssid"] = ssid;
    wifiData["password"] = password;
    
    Json::StreamWriterBuilder writer;
    std::string jsonData = Json::writeString(writer, wifiData);
    
    std::string url = "http://" + m_ip + ":5000/system/connectWifiJson";
    std::string response = sendPostRequest(url, jsonData);
    
    Json::Value root;
    if (!parseResponse(response, root)) {
        if (callback) callback(-1, "Failed to parse response");
        return false;
    }
    
    int code = root["code"].asInt();
    std::string message = root["message"].asString();
    
    if (callback) callback(code, message);
    return (code == 0);
}

bool TBotSDK::uploadCustomData(const std::string& name, const std::string& jsonData, StatusCallback callback) {
    std::string url = "http://" + m_ip + ":5000/upload_json/" + name;
    std::string response = sendPostRequest(url, jsonData);
    
    Json::Value root;
    if (!parseResponse(response, root)) {
        if (callback) callback(-1, "Failed to parse response");
        return false;
    }
    
    int code = root["code"].asInt();
    std::string message = root["message"].asString();
    
    if (callback) callback(code, message);
    return (code == 0);
}

std::string TBotSDK::downloadCustomData(const std::string& name, StatusCallback callback) {
    std::string url = "http://" + m_ip + ":5000/download_json/" + name;
    std::string response = sendGetRequest(url);
    
    Json::Value root;
    if (!parseResponse(response, root)) {
        if (callback) callback(-1, "Failed to parse response");
        return "";
    }
    
    if (callback) callback(root["code"].asInt(), "Success");
    
    Json::StreamWriterBuilder writer;
    return Json::writeString(writer, root["data"]);
}

bool TBotSDK::deleteCustomData(const std::string& name, StatusCallback callback) {
    std::string url = "http://" + m_ip + ":5000/delete_json/" + name;
    std::string response = sendGetRequest(url);
    
    Json::Value root;
    if (!parseResponse(response, root)) {
        if (callback) callback(-1, "Failed to parse response");
        return false;
    }
    
    int code = root["code"].asInt();
    std::string message = root["message"].asString();
    
    if (callback) callback(code, message);
    return (code == 0);
}

bool TBotSDK::getEditedMapData(const std::string& mapName, std::vector<int>& path, std::vector<int>& wall, StatusCallback callback) {
    std::string url = "http://" + m_ip + ":5000/get_wall_path/" + mapName;
    std::string response = sendGetRequest(url);
    
    Json::Value root;
    if (!parseResponse(response, root)) {
        if (callback) callback(-1, "Failed to parse response");
        return false;
    }
    
    int code = root["code"].asInt();
    if (code == 0) {
        Json::Value data = root["data"];
        for (const auto& item : data["path"]) {
            path.push_back(item.asInt());
        }
        for (const auto& item : data["wall"]) {
            wall.push_back(item.asInt());
        }
    }
    
    if (callback) callback(code, code == 0 ? "Success" : "Failed");
    return (code == 0);
}

bool TBotSDK::saveEditedMap(const std::string& mapName, const std::vector<int>& path, const std::vector<int>& wall, StatusCallback callback) {
    Json::Value mapData;
    Json::Value pathData;
    Json::Value wallData;
    
    for (const auto& item : path) {
        pathData.append(item);
    }
    for (const auto& item : wall) {
        wallData.append(item);
    }
    
    mapData["path"] = pathData;
    mapData["wall"] = wallData;
    
    Json::StreamWriterBuilder writer;
    std::string jsonData = Json::writeString(writer, mapData);
    
    std::string url = "http://" + m_ip + ":5000/save_mask/" + mapName;
    std::string response = sendPostRequest(url, jsonData);
    
    Json::Value root;
    if (!parseResponse(response, root)) {
        if (callback) callback(-1, "Failed to parse response");
        return false;
    }
    
    int code = root["code"].asInt();
    
    if (callback) callback(code, code == 0 ? "Success" : "Failed");
    return (code == 0);
}

// 位置点管理功能实现
bool TBotSDK::saveWaypoint(const std::string& name, const Pose& pose, const std::string& description, const std::string& mapName, StatusCallback callback) {
    if (!m_connected) {
        if (callback) callback(-1, "Not connected to robot");
        return false;
    }
    
    // 构建位置点数据
    Json::Value waypointData;
    waypointData["name"] = name;
    waypointData["pose"]["position"]["x"] = pose.position.x;
    waypointData["pose"]["position"]["y"] = pose.position.y;
    waypointData["pose"]["position"]["z"] = pose.position.z;
    waypointData["pose"]["orientation"]["x"] = pose.orientation.x;
    waypointData["pose"]["orientation"]["y"] = pose.orientation.y;
    waypointData["pose"]["orientation"]["z"] = pose.orientation.z;
    waypointData["pose"]["orientation"]["w"] = pose.orientation.w;
    waypointData["description"] = description;
    waypointData["map_name"] = mapName;
    
    // 获取当前时间
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S");
    waypointData["created_time"] = ss.str();
    
    Json::StreamWriterBuilder writer;
    std::string jsonData = Json::writeString(writer, waypointData);
    
    // 使用自定义数据存储位置点
    std::string dataName = "waypoint_" + name;
    return uploadCustomData(dataName, jsonData, callback);
}

bool TBotSDK::saveCurrentPositionAsWaypoint(const std::string& name, const std::string& description, StatusCallback callback) {
    if (!m_connected) {
        if (callback) callback(-1, "Not connected to robot");
        return false;
    }
    
    // 获取当前机器人位置
    Pose currentPose;
    bool poseReceived = false;
    
    getRobotDataStream([&](const RobotData& data) {
        currentPose = data.robot_pose;
        poseReceived = true;
    });
    
    if (!poseReceived) {
        if (callback) callback(-1, "Failed to get current position");
        return false;
    }
    
    // 保存当前位置作为位置点
    return saveWaypoint(name, currentPose, description, "", callback);
}

bool TBotSDK::loadWaypoint(const std::string& name, WaypointCallback callback) {
    if (!m_connected) {
        if (callback) callback(Waypoint{});
        return false;
    }
    
    std::string dataName = "waypoint_" + name;
    std::string jsonData = downloadCustomData(dataName);
    
    if (jsonData.empty()) {
        if (callback) callback(Waypoint{});
        return false;
    }
    
    // 解析JSON数据
    Json::Value root;
    if (!parseResponse(jsonData, root)) {
        if (callback) callback(Waypoint{});
        return false;
    }
    
    // 构建位置点对象
    Waypoint waypoint;
    waypoint.name = root["name"].asString();
    waypoint.pose.position.x = root["pose"]["position"]["x"].asFloat();
    waypoint.pose.position.y = root["pose"]["position"]["y"].asFloat();
    waypoint.pose.position.z = root["pose"]["position"]["z"].asFloat();
    waypoint.pose.orientation.x = root["pose"]["orientation"]["x"].asFloat();
    waypoint.pose.orientation.y = root["pose"]["orientation"]["y"].asFloat();
    waypoint.pose.orientation.z = root["pose"]["orientation"]["z"].asFloat();
    waypoint.pose.orientation.w = root["pose"]["orientation"]["w"].asFloat();
    waypoint.description = root["description"].asString();
    waypoint.map_name = root["map_name"].asString();
    waypoint.created_time = root["created_time"].asString();
    
    if (callback) callback(waypoint);
    return true;
}

std::vector<Waypoint> TBotSDK::getAllWaypoints(const std::string& mapName, StatusCallback callback) {
    std::vector<Waypoint> waypoints;
    
    if (!m_connected) {
        if (callback) callback(-1, "Not connected to robot");
        return waypoints;
    }
    
    // 获取所有自定义数据名称
    std::string dataList = downloadCustomData("waypoint_list");
    if (dataList.empty()) {
        // 尝试获取所有位置点（这里简化处理，实际可能需要更复杂的逻辑）
        std::vector<std::string> commonNames = {"A点", "B点", "C点", "D点", "E点", "F点", "G点", "H点"};
        
        for (const auto& name : commonNames) {
            Waypoint waypoint;
            if (loadWaypoint(name, [&](const Waypoint& wp) {
                waypoint = wp;
            })) {
                if (mapName.empty() || waypoint.map_name == mapName) {
                    waypoints.push_back(waypoint);
                }
            }
        }
    } else {
        // 解析位置点列表
        Json::Value root;
        if (parseResponse(dataList, root)) {
            for (const auto& name : root) {
                Waypoint waypoint;
                if (loadWaypoint(name.asString(), [&](const Waypoint& wp) {
                    waypoint = wp;
                })) {
                    if (mapName.empty() || waypoint.map_name == mapName) {
                        waypoints.push_back(waypoint);
                    }
                }
            }
        }
    }
    
    if (callback) callback(0, "Successfully loaded " + std::to_string(waypoints.size()) + " waypoints");
    return waypoints;
}

bool TBotSDK::deleteWaypoint(const std::string& name, StatusCallback callback) {
    if (!m_connected) {
        if (callback) callback(-1, "Not connected to robot");
        return false;
    }
    
    std::string dataName = "waypoint_" + name;
    return deleteCustomData(dataName, callback);
}

bool TBotSDK::navigateToWaypoint(const std::string& name, StatusCallback callback) {
    if (!m_connected) {
        if (callback) callback(-1, "Not connected to robot");
        return false;
    }
    
    // 加载位置点
    Waypoint waypoint;
    bool waypointLoaded = false;
    
    loadWaypoint(name, [&](const Waypoint& wp) {
        waypoint = wp;
        waypointLoaded = true;
    });
    
    if (!waypointLoaded || waypoint.name.empty()) {
        if (callback) callback(-1, "Waypoint not found: " + name);
        return false;
    }
    
    // 导航到位置点
    return navigateTo(waypoint.pose.position.x, waypoint.pose.position.y, 
                     waypoint.pose.position.z, waypoint.pose.orientation.w, callback);
}

bool TBotSDK::navigateToWaypointAndWait(const std::string& name, int timeout_sec, StatusCallback callback) {
    if (!m_connected) {
        if (callback) callback(-1, "Not connected to robot");
        return false;
    }
    
    // 加载位置点
    Waypoint waypoint;
    bool waypointLoaded = false;
    
    loadWaypoint(name, [&](const Waypoint& wp) {
        waypoint = wp;
        waypointLoaded = true;
    });
    
    if (!waypointLoaded || waypoint.name.empty()) {
        if (callback) callback(-1, "Waypoint not found: " + name);
        return false;
    }
    
    // 同步导航到位置点
    return navigateToAndWait(waypoint.pose.position.x, waypoint.pose.position.y, 
                           waypoint.pose.position.z, waypoint.pose.orientation.w, timeout_sec, callback);
}

} // namespace TBot