
# cmake version
cmake_minimum_required(VERSION 3.28.3)
set(CMAKE_POLICY_VERSION_MINIMUM 3.5)

# project name
project(BasicService VERSION 1.0.0.0 LANGUAGES CXX)

# 查找要编译的.cpp文件
# search source files for .cpp 
file(GLOB_RECURSE OTA_ENGINE_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
)

# 构建时间
# build time
string(TIMESTAMP APP_BUILD_TIME "%Y-%m-%d %H:%M:%S")

# 根据version.h.in模板生产version.h头文件
# generate version.h from version.h.in
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/version.h.in ${CMAKE_CURRENT_SOURCE_DIR}/version.h)

# EC-Master SDK/Sources 内置到仓库，仍可通过 CMake 变量覆盖
set(ECM_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/third_party/ecmaster_sdk" CACHE PATH "Path to EC-Master SDK root")
set(ECM_SOURCE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/third_party/ecmaster_sdk/Sources" CACHE PATH "Path to EC-Master Sources root")
set(ECM_SDK_LIB_DIR "${ECM_SDK_ROOT}/LIB/Linux/aarch64")
set(ECM_LL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/ecmaster_lib" CACHE PATH "Path to EC-Master link-layer .so directory")

add_executable(${PROJECT_NAME} ${OTA_ENGINE_SRC})
message("app PROJECT_NAME:" ${PROJECT_NAME})

include(FetchContent)
# add dependence library common_util
FetchContent_Declare(common_util
    GIT_REPOSITORY https://github.com/spencer-luo/common_util.git
    GIT_TAG v1.4.1)
FetchContent_MakeAvailable(common_util)
# add dependence library yaml-cpp (patch cmake_minimum_required 3.4->3.5 for modern CMake)
FetchContent_Declare(yaml-cpp
    GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
    GIT_TAG 0.8.0
    PATCH_COMMAND sed -i "s/VERSION 3.4/VERSION 3.5/" CMakeLists.txt)
FetchContent_MakeAvailable(yaml-cpp)
# add dependence library tiny_framework
FetchContent_Declare(tiny_framework
    GIT_REPOSITORY git@github.com:New-Stark-Industries/tiny_framework.git
    GIT_TAG master)
FetchContent_MakeAvailable(tiny_framework)

target_link_libraries(${PROJECT_NAME} PRIVATE
    common_util
    yaml-cpp
    tiny_framework
    ecmaster_demo
    EcMaster
    EcMasterRasServer
    pthread
    m
    dl
    rt
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
    # $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> # 构建时使用源码目录的 include
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}> # 安装后使用安装目录的 include
    PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    ${ECM_SDK_ROOT}/INC
    ${ECM_SDK_ROOT}/INC/Linux
    ${ECM_SOURCE_ROOT}/Common
    ${ECM_SOURCE_ROOT}/LinkOsLayer
)

target_link_directories(${PROJECT_NAME} PRIVATE
    ${ECM_SDK_LIB_DIR}
)

# Runtime search path for link-layer .so (e.g. libemllDW3504.so)
set_target_properties(${PROJECT_NAME} PROPERTIES
    BUILD_RPATH "$ORIGIN/../third_party/ecmaster_lib"
    INSTALL_RPATH "$ORIGIN/../third_party/ecmaster_lib"
)

add_subdirectory(third_party/ecmaster_demo)
