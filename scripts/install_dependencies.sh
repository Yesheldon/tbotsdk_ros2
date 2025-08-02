#!/bin/bash

# TBot SDK 依赖安装脚本
# 使用方法: ./scripts/install_dependencies.sh

set -e

echo "=========================================="
echo "TBot SDK 依赖安装脚本"
echo "=========================================="

# 检测操作系统
if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    # Linux
    if command -v apt-get &> /dev/null; then
        echo "检测到 Ubuntu/Debian 系统"
        echo "安装系统依赖..."
        sudo apt-get update
        sudo apt-get install -y \
            build-essential \
            cmake \
            git \
            libcurl4-openssl-dev \
            libjsoncpp-dev \
            zlib1g-dev \
            python3-pip \
            pkg-config
    elif command -v yum &> /dev/null; then
        echo "检测到 CentOS/RHEL 系统"
        echo "安装系统依赖..."
        sudo yum update
        sudo yum install -y \
            gcc-c++ \
            cmake \
            git \
            libcurl-devel \
            jsoncpp-devel \
            zlib-devel \
            python3-pip \
            pkgconfig
    else
        echo "不支持的Linux发行版，请手动安装依赖"
        exit 1
    fi
elif [[ "$OSTYPE" == "darwin"* ]]; then
    # macOS
    echo "检测到 macOS 系统"
    if command -v brew &> /dev/null; then
        echo "使用 Homebrew 安装依赖..."
        brew update
        brew install curl jsoncpp zlib cmake
    else
        echo "请先安装 Homebrew: https://brew.sh/"
        exit 1
    fi
else
    echo "不支持的操作系统: $OSTYPE"
    exit 1
fi

# 检查ROS2
echo "检查ROS2环境..."
if ! command -v ros2 &> /dev/null; then
    echo "警告: 未检测到ROS2，请先安装ROS2"
    echo "安装指南: https://docs.ros.org/en/humble/Installation.html"
else
    echo "ROS2已安装: $(ros2 --version)"
fi

# 检查依赖
echo "检查依赖库..."
if pkg-config --exists libcurl; then
    echo "✓ CURL库已安装"
else
    echo "✗ CURL库未找到"
fi

if pkg-config --exists jsoncpp; then
    echo "✓ JSON库已安装"
else
    echo "✗ JSON库未找到"
fi

if pkg-config --exists zlib; then
    echo "✓ ZLIB库已安装"
else
    echo "✗ ZLIB库未找到"
fi

echo "=========================================="
echo "依赖安装完成！"
echo "=========================================="
echo ""
echo "下一步:"
echo "1. 确保ROS2环境已设置: source /opt/ros/humble/setup.bash"
echo "2. 编译项目: colcon build --packages-select tbot_sdk"
echo "3. 运行测试: ./scripts/quick_test.sh" 