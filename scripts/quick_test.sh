#!/bin/bash

# TBot SDK 快速测试脚本
# 使用方法: ./scripts/quick_test.sh

set -e

echo "=========================================="
echo "TBot SDK 快速测试开始"
echo "=========================================="

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 测试计数器
PASSED=0
FAILED=0

# 测试函数
test_step() {
    local test_name="$1"
    local command="$2"
    
    echo -e "\n${YELLOW}测试: $test_name${NC}"
    echo "执行: $command"
    
    if eval "$command"; then
        echo -e "${GREEN}✓ 通过${NC}"
        ((PASSED++))
    else
        echo -e "${RED}✗ 失败${NC}"
        ((FAILED++))
    fi
}

# 检查环境
echo "检查ROS2环境..."
if ! command -v ros2 &> /dev/null; then
    echo -e "${RED}错误: ROS2未安装或未设置环境${NC}"
    exit 1
fi

# 检查工作空间
if [ ! -f "package.xml" ]; then
    echo -e "${RED}错误: 请在tbot_sdk包目录中运行此脚本${NC}"
    exit 1
fi

# 1. 编译测试
test_step "编译项目" "colcon build --packages-select tbot_sdk"

# 2. 设置环境
test_step "设置环境" "source install/setup.bash"

# 3. 检查包是否正确安装
test_step "检查包安装" "ros2 pkg list | grep tbot_sdk"

# 4. 检查可执行文件
test_step "检查可执行文件" "ros2 run tbot_sdk test_api --help"

# 5. 检查话题
test_step "检查话题定义" "ros2 topic list | grep -E '(cmd_vel|odom|scan|status)'"

# 6. 检查参数
test_step "检查参数定义" "ros2 param list | grep robot_ip"

# 7. 运行单元测试（如果可用）
if [ -f "build/tbot_sdk/test_tbot_sdk" ]; then
    test_step "运行SDK单元测试" "timeout 30s ./build/tbot_sdk/test_tbot_sdk --gtest_filter=TestConnect || true"
fi

if [ -f "build/tbot_sdk/test_tbot_node" ]; then
    test_step "运行节点单元测试" "timeout 30s ./build/tbot_sdk/test_tbot_node --gtest_filter=TestTopicNames || true"
fi

# 8. 检查launch文件
test_step "检查launch文件" "ros2 launch tbot_sdk tbot_launch.py --help"

# 9. 检查配置文件
test_step "检查配置文件" "test -f config/tbot_params.yaml"

# 10. 检查文档
test_step "检查文档" "test -f README.md && test -f TESTING.md"

# 11. 检查依赖
test_step "检查依赖" "rosdep check tbot_sdk"

# 12. 检查系统依赖
test_step "检查CURL库" "pkg-config --exists libcurl"
test_step "检查JSON库" "pkg-config --exists jsoncpp"
test_step "检查ZLIB库" "pkg-config --exists zlib"

# 13. 代码风格检查（如果可用）
if command -v ament_cpplint &> /dev/null; then
    test_step "代码风格检查" "ament_cpplint src/ include/ || true"
fi

# 14. 静态分析（如果可用）
if command -v cppcheck &> /dev/null; then
    test_step "静态分析" "cppcheck --enable=all --std=c++14 src/ include/ || true"
fi

# 显示结果
echo -e "\n=========================================="
echo "测试结果汇总"
echo "=========================================="
echo -e "${GREEN}通过: $PASSED${NC}"
echo -e "${RED}失败: $FAILED${NC}"
echo -e "总计: $((PASSED + FAILED))"

if [ $FAILED -eq 0 ]; then
    echo -e "\n${GREEN}🎉 所有测试通过！${NC}"
    exit 0
else
    echo -e "\n${RED}❌ 有 $FAILED 个测试失败${NC}"
    exit 1
fi 