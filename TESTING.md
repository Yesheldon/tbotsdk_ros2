# TBot SDK 测试和使用指南

## 目录
1. [测试结构概述](#测试结构概述)
2. [编译和安装](#编译和安装)
3. [静态功能测试](#静态功能测试)
4. [动态功能测试](#动态功能测试)
5. [测试运行方法](#测试运行方法)
6. [性能测试](#性能测试)
7. [故障排除](#故障排除)

## 测试结构概述

### 重新组织的测试结构

根据功能分类，测试文件已重新组织为两大类，统一放在 `test` 目录下：

#### 📁 目录结构
```
test/
├── static/                    # 静态功能测试
│   ├── test_connection.cpp    # 连接状态测试
│   ├── test_data_stream.cpp  # 数据流测试
│   ├── test_robot_status.cpp # 机器人状态测试
│   └── test_map_status.cpp   # 地图状态测试
├── dynamic/                   # 动态功能测试
│   ├── test_navigation.cpp   # 导航功能测试
│   ├── test_velocity_control.cpp # 速度控制测试
│   └── test_waypoints.cpp    # 位置点管理测试
├── CMakeLists.txt            # 测试编译配置
├── run_all_tests.sh          # 运行所有测试脚本
├── run_static_tests.sh       # 运行静态测试脚本
├── run_dynamic_tests.sh      # 运行动态测试脚本
├── README.md                 # 测试说明文档
└── TEST_STRUCTURE.md         # 测试结构总结
```

#### 🔧 功能分类

**静态功能测试** (`static/`)
- 数据收集和状态监控功能
- 不涉及机器人的实际运动
- 使用Mock对象进行单元测试

**动态功能测试** (`dynamic/`)
- 机器人的运动控制和导航功能
- 测试速度控制、导航、位置点管理等
- 使用Mock对象模拟实际行为

#### ⚠️ 排除功能
- **建图功能**: 由上位机处理，不包含在SDK测试中

#### 🎯 测试特性
- **Mock测试**: 使用Google Mock框架，无需实际硬件
- **全面覆盖**: 覆盖SDK的所有主要功能
- **中文支持**: 测试用例支持中文位置点名称
- **异步回调**: 测试异步操作和回调机制
- **错误处理**: 包含各种错误场景的测试

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

# 安装Google Test和Google Mock
sudo apt-get install -y \
    libgtest-dev \
    libgmock-dev
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

# 编译（包含测试）
colcon build --packages-select tbot_sdk --cmake-args -DBUILD_TESTING=ON

# 设置环境
source install/setup.bash
```

## 静态功能测试

### 1. 连接状态测试 (`test_connection.cpp`)

测试机器人的连接相关功能：

```bash
# 编译测试
cd test
mkdir -p build && cd build
cmake ..
make test_connection

# 运行连接测试
./test_connection
```

**测试内容**:
- 连接成功/失败场景
- 连接状态检查
- 服务状态检查
- 断开连接功能
- 参数验证（无效IP等）

### 2. 数据流测试 (`test_data_stream.cpp`)

测试机器人数据和地图数据流：

```bash
# 运行数据流测试
./test_data_stream
```

**测试内容**:
- 机器人数据流获取
- 地图数据流获取
- 数据内容验证
- 连续数据接收
- 错误处理

### 3. 机器人状态测试 (`test_robot_status.cpp`)

测试各种机器人状态信息：

```bash
# 运行机器人状态测试
./test_robot_status
```

**测试内容**:
- 获取机器人状态
- 获取电池状态
- 获取系统状态
- 状态数据验证
- 边界条件测试

### 4. 地图状态测试 (`test_map_status.cpp`)

测试地图相关的状态和信息：

```bash
# 运行地图状态测试
./test_map_status
```

**测试内容**:
- 获取地图列表
- 获取当前地图
- 获取地图状态
- 地图内容验证
- 地图分辨率检查

## 动态功能测试

### 1. 导航功能测试 (`test_navigation.cpp`)

测试机器人的导航能力：

```bash
# 运行导航测试
./test_navigation
```

**测试内容**:
- 启动导航
- 导航到指定坐标
- 同步导航等待
- 停止导航
- 暂停/恢复导航
- 导航参数验证

### 2. 速度控制测试 (`test_velocity_control.cpp`)

测试速度控制功能：

```bash
# 运行速度控制测试
./test_velocity_control
```

**测试内容**:
- 速度控制命令
- 设置最大速度
- 停止命令
- 急停功能
- 各种运动模式
- 速度限制验证

### 3. 位置点管理测试 (`test_waypoints.cpp`)

测试位置点管理功能：

```bash
# 运行位置点测试
./test_waypoints
```

**测试内容**:
- 保存当前位置点
- 加载位置点信息
- 获取所有位置点
- 导航到位置点
- 删除位置点
- 中文位置点名称支持

## 测试运行方法

### 1. 运行所有测试

```bash
# 使用测试脚本运行所有测试
chmod +x test/run_all_tests.sh
./test/run_all_tests.sh
```

### 2. 运行分类测试

```bash
# 只运行静态功能测试
chmod +x test/run_static_tests.sh
./test/run_static_tests.sh

# 只运行动态功能测试
chmod +x test/run_dynamic_tests.sh
./test/run_dynamic_tests.sh
```

### 3. 运行单个测试

```bash
# 进入build目录
cd build

# 运行连接测试
./test/test_connection

# 运行导航测试
./test/test_navigation

# 运行数据流测试
./test/test_data_stream

# 运行机器人状态测试
./test/test_robot_status

# 运行地图状态测试
./test/test_map_status

# 运行速度控制测试
./test/test_velocity_control

# 运行位置点测试
./test/test_waypoints
```

### 4. 使用ROS2测试框架

```bash
# 编译并运行ROS2测试
colcon build --packages-select tbot_sdk --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select tbot_sdk --event-handlers console_direct+

# 查看测试结果
colcon test-result --all
```

### 5. 测试覆盖率

```bash
# 安装覆盖率工具
sudo apt-get install -y lcov

# 编译时启用覆盖率
colcon build --packages-select tbot_sdk --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Debug

# 运行测试并生成覆盖率报告
lcov --capture --directory build/tbot_sdk --output-file coverage.info
genhtml coverage.info --output-directory coverage_report
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

# 运行静态测试
echo "Running static tests..."
./test/run_static_tests.sh

# 运行动态测试
echo "Running dynamic tests..."
./test/run_dynamic_tests.sh

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

# 错误：找不到Google Test
sudo apt-get install libgtest-dev libgmock-dev

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
./build/tbot_sdk/test_connection --gtest_filter=ConnectionTest.TestConnectSuccess

# 调试模式运行
gdb ./build/tbot_sdk/test_connection
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
          libjsoncpp-dev \
          libgtest-dev \
          libgmock-dev
    
    - name: Install ROS2
      run: |
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
    
    - name: Build and test
      run: |
        source /opt/ros/humble/setup.sh
        colcon build --packages-select tbot_sdk --cmake-args -DBUILD_TESTING=ON
        ./test/run_all_tests.sh
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
./test/run_all_tests.sh

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
6. **Mock测试**: 使用Mock对象确保测试的独立性
7. **分类测试**: 按功能分类组织测试文件
8. **自动化**: 使用脚本自动化测试流程 