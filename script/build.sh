#!/bin/bash

# 获取脚本所在目录的上级目录（项目根目录）
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
BUILD_DIR="$PROJECT_ROOT/build"

# 板子上的运行配置
ECMASTER_LIB_PATH="/home/stark/cai/EC-Master/Bin/Linux/aarch64"
ENI_FILE="${PROJECT_ROOT}/conf/arm-b_eni.xml"
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
    else
        # 缓存里可能仍指向已删除的 _deps/tiny_framework-src，清除后让 FetchContent 重新拉取
        CMAKE_ARGS="-U FETCHCONTENT_SOURCE_DIR_TINY_FRAMEWORK"
    fi
    cmake .. $CMAKE_ARGS
    # 3. 编译（开启多线程加速）
    make -j$(nproc)
elif [ "$1" = "r" ]; then
    # 运行 EcMasterDemoDc
    echo "=== 运行 EcMasterDemoDc ==="
    cd "$DEMO_BIN_DIR"
    chmod +x EcMasterDemoDc
    sudo LD_LIBRARY_PATH=${ECMASTER_LIB_PATH} ./EcMasterDemoDc \
        -dw3504 2 1 custom rk3588s osdriver 0 \
        -f ${ENI_FILE} \
        -lic 00000080-7FFFC2C3-47698409-335975A0-E8FE19E2-EAE2CF55 \
        -t 0
elif [ "$1" = "c" ]; then
    rm -rf "$BUILD_DIR"
elif [ "$1" = "f" ]; then
    # 将配置cfg.json文件复制到basic_service目录下
    cp conf/cfg.json    $HOME/cai/basic_service/conf/
    echo "=== 配置文件复制完成 ==="
else
    echo "Usage: $0 {b|r|c}"
    echo "  b - Build (compile on board)"
    echo "  r - Run EcMasterDemoDc"
    echo "  c - Clean build directory"
fi
