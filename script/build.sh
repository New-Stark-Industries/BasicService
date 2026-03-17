#!/bin/bash

# 获取脚本所在目录的上级目录（项目根目录）
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
BUILD_DIR="$PROJECT_ROOT/build"

# 板子上的运行配置
ECMASTER_LIB_PATH="/home/stark/src/EC-Master/Bin/Linux/aarch64"
ENI_FILE="${PROJECT_ROOT}/conf/eni.xml"
DEMO_BIN_DIR="${BUILD_DIR}/third_party/ecmaster_demo"

cd "$PROJECT_ROOT"

if [ "$1" = "b" ]; then
    # 1. 创建编译目录
    mkdir -p "$BUILD_DIR" && cd "$BUILD_DIR"
    # 2. 生成 Makefile（本地编译，不需要交叉编译工具链）
    # 如果本地有 tiny_framework 则使用，否则让 CMake 从 git 拉取
    CMAKE_ARGS=""
    if [ -d "$HOME/src/tiny_framework" ]; then
        CMAKE_ARGS="-DFETCHCONTENT_SOURCE_DIR_TINY_FRAMEWORK=$HOME/src/tiny_framework"
    elif [ -d "$HOME/tiny_framework" ]; then
        CMAKE_ARGS="-DFETCHCONTENT_SOURCE_DIR_TINY_FRAMEWORK=$HOME/tiny_framework"
    elif [ -d "$BUILD_DIR/_deps/tiny_framework-src" ]; then
        # 已缓存的依赖，直接指向它，避免 cmake 重新 fetch（网络不通时会卡住）
        CMAKE_ARGS="-DFETCHCONTENT_SOURCE_DIR_TINY_FRAMEWORK=$BUILD_DIR/_deps/tiny_framework-src"
    fi
    cmake .. $CMAKE_ARGS
    # 3. 编译（开启多线程加速）
    make -j$(nproc)
elif [ "$1" = "r" ]; then
    # 运行 BasicService（含 EtherCAT demo，配置来自 busi.yaml）
    echo "=== 运行 BasicService ==="
    cd "$PROJECT_ROOT"
    sudo LD_LIBRARY_PATH=${ECMASTER_LIB_PATH} \
        "${BUILD_DIR}/BasicService" \
        "${PROJECT_ROOT}/conf/app.yaml" \
        "${PROJECT_ROOT}/conf/busi.yaml"
elif [ "$1" = "d" ]; then
    # 直接运行 EcMasterDemoDc（跳过 BasicService，用于底层调试）
    echo "=== 运行 EcMasterDemoDc ==="
    cd "$DEMO_BIN_DIR"
    chmod +x EcMasterDemoDc
    sudo LD_LIBRARY_PATH=${ECMASTER_LIB_PATH} ./EcMasterDemoDc \
        -dw3504 2 1 custom rk3588s osdriver 0 \
        -f "${ENI_FILE}" \
        -t 0
elif [ "$1" = "c" ]; then
    rm -rf "$BUILD_DIR"
elif [ "$1" = "f" ]; then
    # 将配置cfg.json文件复制到basic_service目录下
    cp conf/cfg.json    $HOME/src/basic_service/conf/
    echo "=== 配置文件复制完成 ==="
else
    echo "Usage: $0 {b|r|d|c}"
    echo "  b - Build (compile on board)"
    echo "  r - Run BasicService (reads conf/busi.yaml, supports motor_map)"
    echo "  d - Run EcMasterDemoDc directly (for low-level debug)"
    echo "  c - Clean build directory"
fi
