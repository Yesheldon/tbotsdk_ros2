# TBot SDK ROS2 Package

这是一个将TBot机器人SDK集成到ROS2系统的包。

## 功能特性

- 机器人连接和状态管理
- 里程计数据发布
- 激光雷达数据发布
- 速度控制订阅
- 导航目标订阅
- TF变换广播

## 依赖项

### 系统依赖
```bash
# Ubuntu/Debian
sudo apt-get install libcurl4-openssl-dev libjsoncpp-dev zlib1g-dev

# 或者使用vcpkg (Windows)
vcpkg install curl jsoncpp zlib
```

### ROS2依赖
- rclcpp
- std_msgs
- geometry_msgs
- sensor_msgs
- nav_msgs
- tf2
- tf2_ros

## 编译

1. 将包放入ROS2工作空间的src目录：
```bash
cd ~/ros2_ws/src
git clone <your-repo-url> tbot_sdk
```

2. 安装依赖：
```bash
# 自动安装依赖（推荐）
chmod +x scripts/install_dependencies.sh
./scripts/install_dependencies.sh

# 或者手动安装
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

3. 编译：
```bash
colcon build --packages-select tbot_sdk
```

## 使用方法

### 启动节点
```bash
# 使用launch文件启动
ros2 launch tbot_sdk tbot_launch.py

# 或者直接启动节点
ros2 run tbot_sdk tbot_node
```

### 设置机器人IP
```bash
ros2 launch tbot_sdk tbot_launch.py robot_ip:=192.168.8.110
```

### 控制机器人

#### 速度控制
```bash
# 发布速度命令
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

#### 导航控制
```bash
# 发布导航目标
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'map'}, pose: {position: {x: 5.0, y: 3.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

### 查看数据

#### 查看里程计数据
```bash
ros2 topic echo /odom
```

#### 查看激光雷达数据
```bash
ros2 topic echo /scan
```

#### 查看机器人状态
```bash
ros2 topic echo /tbot/status
```

#### 查看TF变换
```bash
ros2 run tf2_tools view_frames
```

## 话题

### 发布的话题
- `/odom` (nav_msgs/Odometry): 里程计数据
- `/scan` (sensor_msgs/LaserScan): 激光雷达数据
- `/tbot/status` (std_msgs/String): 机器人状态信息

### 订阅的话题
- `/cmd_vel` (geometry_msgs/Twist): 速度控制命令
- `/goal_pose` (geometry_msgs/PoseStamped): 导航目标

## 参数

- `robot_ip` (string): 机器人IP地址，默认 "192.168.8.110"

## 测试

### 快速测试
```bash
# 运行快速测试脚本
chmod +x scripts/quick_test.sh
./scripts/quick_test.sh
```

### 单元测试
```bash
# 编译并运行单元测试
colcon build --packages-select tbot_sdk --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select tbot_sdk --event-handlers console_direct+

# 查看测试结果
colcon test-result --all
```

### 集成测试
```bash
# 运行Python集成测试
python3 test/integration_test.py

# 运行测试程序
ros2 run tbot_sdk test_api

# 运行数据流测试
ros2 run tbot_sdk test_data_stream

# 运行超时机制测试
ros2 run tbot_sdk test_timeout

# 运行位置点管理测试
ros2 run tbot_sdk test_waypoints

# 启动位置点服务
ros2 run tbot_sdk waypoint_service
```

### 详细测试说明
请参考 [TESTING.md](TESTING.md) 文件获取完整的测试指南。

## 位置点管理

SDK提供了完整的位置点管理功能，支持保存、加载、删除和导航到位置点：

### 基本功能
- **保存位置点**: `saveWaypoint()` - 保存指定位置和姿态
- **保存当前位置**: `saveCurrentPositionAsWaypoint()` - 保存机器人当前位置
- **加载位置点**: `loadWaypoint()` - 加载指定位置点信息
- **获取所有位置点**: `getAllWaypoints()` - 获取所有保存的位置点
- **删除位置点**: `deleteWaypoint()` - 删除指定位置点
- **导航到位置点**: `navigateToWaypoint()` - 异步导航到位置点
- **同步导航到位置点**: `navigateToWaypointAndWait()` - 同步导航并等待完成

### 使用示例
```cpp
// 保存当前位置作为A点
tbot.saveCurrentPositionAsWaypoint("A点", "起始位置");

// 导航到A点
tbot.navigateToWaypoint("A点");

// 同步导航到B点并等待完成
tbot.navigateToWaypointAndWait("B点", 60);
```

### 位置点信息
每个位置点包含：
- **名称**: 如 "A点", "B点", "C点"
- **位置和姿态**: 精确的坐标和朝向信息
- **描述**: 位置点的说明文字
- **地图名称**: 所属的地图
- **创建时间**: 位置点的创建时间戳

### ROS2位置点服务
SDK还提供了ROS2服务接口来管理位置点：

```bash
# 启动位置点服务
ros2 run tbot_sdk waypoint_service

# 保存当前位置点
ros2 service call /save_waypoint std_srvs/srv/Trigger

# 加载所有位置点
ros2 service call /load_waypoint std_srvs/srv/Trigger

# 导航到位置点
ros2 service call /navigate_to_waypoint std_srvs/srv/Trigger

# 删除位置点
ros2 service call /delete_waypoint std_srvs/srv/Trigger

# 查看位置点列表话题
ros2 topic echo /waypoint_list
```

## 超时机制

SDK内置了超时机制来防止长时间阻塞：

- **默认超时**: 30秒（适用于大多数API调用）
- **建图操作**: 120秒（`startMapping`）
- **保存地图**: 60秒（`saveMap`）
- **启动导航**: 60秒（`startNavigation`）
- **数据流获取**: 60秒（`getMapStream`, `getRobotDataStream`）
- **状态确认**: 30秒（`confirmStatus`）

如果请求超时，函数会返回失败并记录错误信息。

## 故障排除

1. 确保机器人IP地址正确
2. 检查网络连接
3. 确保机器人服务正在运行
4. 查看日志输出：`ros2 run tbot_sdk tbot_node --ros-args --log-level debug`
5. 如果遇到超时问题，检查网络延迟或增加超时时间 