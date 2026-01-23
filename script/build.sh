#!/bin/bash

# 获取脚本所在目录的上级目录（项目根目录）
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
BUILD_DIR="$PROJECT_ROOT/build-aarch64"

# 板子配置
BOARD_USER="stark"
BOARD_IP="192.168.31.214"
BOARD_SRC_DIR="~/src/BasicService"
BOARD_BUILD_DIR="~/src/BasicService/build"

# 板子上的运行配置
ECMASTER_LIB_PATH="/home/stark/src/EC-Master/Bin/Linux/aarch64"
ENI_FILE="/home/stark/src/EC-Master/eni.xml"
DEMO_BIN_DIR="${BOARD_BUILD_DIR}/third_party/ecmaster_demo"

cd "$PROJECT_ROOT"

if [ "$1" = "b" ]; then
    # 1. 创建交叉编译目录
    mkdir -p "$BUILD_DIR" && cd "$BUILD_DIR"
    # 2. 生成 Makefile
    cmake .. -DCMAKE_TOOLCHAIN_FILE=../cmake/toolchains/aarch64-linux-gnu.cmake \
             -DFETCHCONTENT_SOURCE_DIR_TINY_FRAMEWORK=~/tiny_framework
    # 3. 编译（开启16线程加速）
    make -j16
elif [ "$1" = "s" ]; then
    # 同步代码和二进制到板子
    echo "=== 同步源码到板子（排除 build* 目录）==="
    rsync -avz --delete --exclude='build*' ./ ${BOARD_USER}@${BOARD_IP}:${BOARD_SRC_DIR}
    
    echo "=== 同步编译好的 aarch64 二进制到板子 ==="
    rsync -avz "$BUILD_DIR/third_party/ecmaster_demo/EcMasterDemoDc" \
               ${BOARD_USER}@${BOARD_IP}:${BOARD_BUILD_DIR}/third_party/ecmaster_demo/
    
    echo "=== 同步完成 ==="
elif [ "$1" = "r" ]; then
    # 在板子上运行程序（通过 SSH）
    echo "=== 在板子上运行 EcMasterDemoDc ==="
    ssh -t ${BOARD_USER}@${BOARD_IP} "cd ${DEMO_BIN_DIR} && \
        chmod +x EcMasterDemoDc && \
        sudo LD_LIBRARY_PATH=${ECMASTER_LIB_PATH} ./EcMasterDemoDc \
            -dw3504 2 1 custom rk3588s osdriver 0 \
            -f ${ENI_FILE} \
            -t 0"
elif [ "$1" = "c" ]; then
    rm -rf "$BUILD_DIR"
else
    echo "Usage: $0 {b|s|r|c}"
    echo "  b - Build (cross-compile for aarch64)"
    echo "  s - Sync code and binary to board"
    echo "  r - Run on board (via SSH)"
    echo "  c - Clean build directory"
fi
