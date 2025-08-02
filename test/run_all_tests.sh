#!/bin/bash

# 运行所有测试脚本
# 包括静态功能和动态功能测试

echo "=========================================="
echo "开始运行所有测试"
echo "=========================================="

# 设置测试目录
TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 运行静态功能测试
echo ""
echo "第一阶段：运行静态功能测试"
echo "------------------------------------------"
if bash "$TEST_DIR/run_static_tests.sh"; then
    echo "✓ 静态功能测试全部通过"
else
    echo "✗ 静态功能测试失败"
    EXIT_CODE=1
fi

echo ""
echo "第二阶段：运行动态功能测试"
echo "------------------------------------------"
if bash "$TEST_DIR/run_dynamic_tests.sh"; then
    echo "✓ 动态功能测试全部通过"
else
    echo "✗ 动态功能测试失败"
    EXIT_CODE=1
fi

echo ""
echo "=========================================="
if [ ${EXIT_CODE:-0} -eq 0 ]; then
    echo "🎉 所有测试通过！"
else
    echo "❌ 部分测试失败，请检查错误信息"
fi
echo "=========================================="

exit ${EXIT_CODE:-0} 