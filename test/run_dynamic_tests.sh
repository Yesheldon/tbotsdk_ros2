#!/bin/bash

# 运行动态功能测试脚本
# 动态功能包括：导航功能、速度控制、导航点管理等

echo "=========================================="
echo "开始运行动态功能测试"
echo "=========================================="

# 设置测试目录
TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="$TEST_DIR/../build"

# 检查构建目录是否存在
if [ ! -d "$BUILD_DIR" ]; then
    echo "错误：构建目录不存在，请先运行 cmake 和 make"
    exit 1
fi

# 进入构建目录
cd "$BUILD_DIR"

echo "运行导航功能测试..."
if ./test/test_navigation; then
    echo "✓ 导航功能测试通过"
else
    echo "✗ 导航功能测试失败"
    EXIT_CODE=1
fi

echo ""
echo "运行速度控制测试..."
if ./test/test_velocity_control; then
    echo "✓ 速度控制测试通过"
else
    echo "✗ 速度控制测试失败"
    EXIT_CODE=1
fi

echo ""
echo "运行导航点管理测试..."
if ./test/test_waypoints; then
    echo "✓ 导航点管理测试通过"
else
    echo "✗ 导航点管理测试失败"
    EXIT_CODE=1
fi

echo ""
echo "=========================================="
echo "动态功能测试完成"
echo "=========================================="

exit ${EXIT_CODE:-0} 