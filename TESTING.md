# TBot SDK 测试和使用指南

## 目录
1. [编译和安装](#编译和安装)
2. [单元测试](#单元测试)
3. [集成测试](#集成测试)
4. [功能测试](#功能测试)
5. [性能测试](#性能测试)
6. [故障排除](#故障排除)

## 编译和安装

### 1. 环境准备

确保您的系统已安装以下依赖：

```bash
# Ubuntu/Debian
sudo apt-get update
sudo apt-get install -y \
    build-essential \
    cmake \
    git \
    libcurl4-openssl-dev \
    libjsoncpp-dev \
    zlib1g-dev \
    python3-pip

# 安装ROS2依赖
sudo apt-get install -y \
    ros-humble-rclcpp \
    ros-humble-std-msgs \
    ros-humble-geometry-msgs \
    ros-humble-sensor-msgs \
    ros-humble-nav-msgs \
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    ros-humble-ament-cmake \
    ros-humble-ament-cmake-gtest \
    ros-humble-ament-cmake-gmock
```

### 2. 编译项目

```bash
# 创建工作空间
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# 克隆项目
git clone <your-repo-url> tbot_sdk
cd tbot_sdk

# 安装依赖
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# 编译
colcon build --packages-select tbot_sdk

# 设置环境
source install/setup.bash
```

## 单元测试

### 1. 运行SDK单元测试

```bash
# 编译测试
colcon build --packages-select tbot_sdk --cmake-args -DBUILD_TESTING=ON

# 运行SDK测试
colcon test --packages-select tbot_sdk --event-handlers console_direct+

# 查看测试结果
colcon test-result --all
```

### 2. 运行节点单元测试

```bash
# 运行节点测试
ros2 run tbot_sdk test_tbot_node

# 或者直接运行
./build/tbot_sdk/test_tbot_node
```

### 3. 测试覆盖率

```bash
# 安装覆盖率工具
sudo apt-get install -y lcov

# 编译时启用覆盖率
colcon build --packages-select tbot_sdk --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Debug

# 运行测试并生成覆盖率报告
lcov --capture --directory build/tbot_sdk --output-file coverage.info
genhtml coverage.info --output-directory coverage_report
```

## 集成测试

### 1. 运行Python集成测试

```bash
# 运行集成测试
python3 test/integration_test.py

# 或者使用unittest
python3 -m unittest test.integration_test -v
```

### 2. 手动集成测试

```bash
# 终端1：启动TBot节点
ros2 launch tbot_sdk tbot_launch.py robot_ip:=192.168.8.110

# 终端2：测试速度控制
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}"

# 终端3：测试导航目标
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'map'}, pose: {position: {x: 1.0, y: 2.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"

# 终端4：查看发布的话题
ros2 topic list
ros2 topic echo /odom
ros2 topic echo /scan
ros2 topic echo /status
```

## 功能测试

### 1. 基本功能测试

```bash
# 测试连接功能
ros2 run tbot_sdk test_api

# 测试参数设置
ros2 param set /tbot_node robot_ip 192.168.8.110
ros2 param get /tbot_node robot_ip

# 测试话题发布和订阅
ros2 topic info /cmd_vel
ros2 topic info /odom
ros2 topic info /scan
```

### 2. 机器人控制测试

```bash
# 测试速度控制
for i in {1..5}; do
    ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}" --once
    sleep 1
done

# 测试停止
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once
```

### 3. 导航功能测试

```bash
# 测试导航到不同位置
positions=(
    "1.0 2.0 0.0 1.0"
    "3.0 1.0 0.0 0.7"
    "0.0 0.0 0.0 1.0"
)

for pos in "${positions[@]}"; do
    read -r x y z w <<< "$pos"
    ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'map'}, pose: {position: {x: $x, y: $y, z: $z}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: $w}}}" --once
    sleep 5
done
```

## 性能测试

### 1. 消息频率测试

```bash
# 测试发布频率
ros2 topic hz /odom
ros2 topic hz /scan
ros2 topic hz /status

# 测试延迟
ros2 topic delay /odom
ros2 topic delay /scan
```

### 2. 内存使用测试

```bash
# 监控内存使用
watch -n 1 'ps aux | grep tbot_node'

# 或者使用htop
htop
```

### 3. CPU使用测试

```bash
# 监控CPU使用
top -p $(pgrep tbot_node)

# 或者使用perf
sudo perf stat -p $(pgrep tbot_node)
```

## 自动化测试脚本

### 1. 创建测试脚本

```bash
#!/bin/bash
# test_runner.sh

echo "Starting TBot SDK tests..."

# 编译测试
colcon build --packages-select tbot_sdk --cmake-args -DBUILD_TESTING=ON

# 运行单元测试
echo "Running unit tests..."
colcon test --packages-select tbot_sdk --event-handlers console_direct+

# 运行集成测试
echo "Running integration tests..."
python3 test/integration_test.py

# 显示测试结果
echo "Test results:"
colcon test-result --all

echo "Tests completed!"
```

### 2. 运行自动化测试

```bash
chmod +x test_runner.sh
./test_runner.sh
```

## 故障排除

### 1. 常见编译错误

```bash
# 错误：找不到CURL库
sudo apt-get install libcurl4-openssl-dev

# 错误：找不到JSON库
sudo apt-get install libjsoncpp-dev

# 错误：找不到ROS2包
sudo apt-get install ros-humble-ament-cmake-gtest
```

### 2. 运行时错误

```bash
# 错误：无法连接到机器人
# 检查网络连接
ping 192.168.8.110

# 检查机器人服务
curl http://192.168.8.110:5000/is_running

# 错误：话题不存在
# 检查节点是否运行
ros2 node list
ros2 topic list
```

### 3. 测试失败

```bash
# 查看详细测试输出
colcon test --packages-select tbot_sdk --event-handlers console_direct+ --verbose

# 运行单个测试
./build/tbot_sdk/test_tbot_sdk --gtest_filter=TestConnect

# 调试模式运行
gdb ./build/tbot_sdk/test_tbot_sdk
```

### 4. 性能问题

```bash
# 检查系统资源
htop
iotop
nethogs

# 检查ROS2性能
ros2 topic hz /odom
ros2 topic bw /odom
```

## 持续集成

### 1. GitHub Actions配置

创建 `.github/workflows/test.yml`:

```yaml
name: TBot SDK Tests

on: [push, pull_request]

jobs:
  test:
    runs-on: ubuntu-20.04
    
    steps:
    - uses: actions/checkout@v2
    
    - name: Install dependencies
      run: |
        sudo apt-get update
        sudo apt-get install -y \
          build-essential \
          cmake \
          libcurl4-openssl-dev \
          libjsoncpp-dev
    
    - name: Install ROS2
      run: |
        sudo apt-get install -y \
          ros-foxy-rclcpp \
          ros-foxy-std-msgs \
          ros-foxy-geometry-msgs \
          ros-foxy-sensor-msgs \
          ros-foxy-nav-msgs \
          ros-foxy-tf2 \
          ros-foxy-tf2-ros \
          ros-foxy-ament-cmake \
          ros-foxy-ament-cmake-gtest
    
    - name: Build and test
      run: |
        source /opt/ros/foxy/setup.sh
        colcon build --packages-select tbot_sdk --cmake-args -DBUILD_TESTING=ON
        colcon test --packages-select tbot_sdk --event-handlers console_direct+
        colcon test-result --all
```

### 2. 本地CI脚本

```bash
#!/bin/bash
# ci.sh

set -e

echo "Starting CI pipeline..."

# 编译
echo "Building..."
colcon build --packages-select tbot_sdk --cmake-args -DBUILD_TESTING=ON

# 运行测试
echo "Running tests..."
colcon test --packages-select tbot_sdk --event-handlers console_direct+

# 检查测试结果
echo "Checking test results..."
colcon test-result --all

# 代码覆盖率
echo "Generating coverage report..."
lcov --capture --directory build/tbot_sdk --output-file coverage.info
genhtml coverage.info --output-directory coverage_report

echo "CI pipeline completed!"
```

## 测试报告

运行测试后，您可以查看以下报告：

1. **单元测试报告**: `build/tbot_sdk/test_results/`
2. **覆盖率报告**: `coverage_report/index.html`
3. **性能报告**: 通过 `ros2 topic hz` 和 `ros2 topic bw` 命令

## 最佳实践

1. **测试驱动开发**: 先写测试，再写代码
2. **持续测试**: 每次提交都运行测试
3. **覆盖率目标**: 保持至少80%的代码覆盖率
4. **性能监控**: 定期检查性能指标
5. **文档更新**: 测试用例也是文档的一部分 