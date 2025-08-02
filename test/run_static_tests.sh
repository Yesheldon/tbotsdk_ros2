#!/bin/bash

# 运行静态功能测试脚本
# 静态功能包括：连接状态、数据流收集、机器人状态、地图状态等

echo "=========================================="
echo "开始运行静态功能测试"
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

echo "运行连接状态测试..."
if ./test/test_connection; then
    echo "✓ 连接状态测试通过"
else
    echo "✗ 连接状态测试失败"
    EXIT_CODE=1
fi

echo ""
echo "运行数据流测试..."
if ./test/test_data_stream; then
    echo "✓ 数据流测试通过"
else
    echo "✗ 数据流测试失败"
    EXIT_CODE=1
fi

echo ""
echo "运行机器人状态测试..."
if ./test/test_robot_status; then
    echo "✓ 机器人状态测试通过"
else
    echo "✗ 机器人状态测试失败"
    EXIT_CODE=1
fi

echo ""
echo "运行地图状态测试..."
if ./test/test_map_status; then
    echo "✓ 地图状态测试通过"
else
    echo "✗ 地图状态测试失败"
    EXIT_CODE=1
fi

echo ""
echo "=========================================="
echo "静态功能测试完成"
echo "=========================================="

exit ${EXIT_CODE:-0} 