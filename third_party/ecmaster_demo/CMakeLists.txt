cmake_minimum_required(VERSION 3.16)

set(ECM_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../ecmaster_sdk" CACHE PATH "Path to EC-Master SDK root")
set(ECM_SOURCE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../ecmaster_sdk/Sources" CACHE PATH "Path to EC-Master Sources root")
set(ECM_SDK_LIB_DIR "${ECM_SDK_ROOT}/LIB/Linux/aarch64")

set(ECM_SOURCES
    EcDemoApp.cpp
    motrotech.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../driver/mcu.cpp
    Common/EcDemoParms.cpp
    Common/EcDemoTimingTask.cpp
    Common/EcLogging.cpp
    Common/EcNotification.cpp
    Common/EcSdoServices.cpp
    Common/EcSelectLinkLayer.cpp
    Common/EcSlaveInfo.cpp
    Common/Linux/EcDemoTimingTaskPlatform.cpp
)

add_library(ecmaster_demo STATIC ${ECM_SOURCES})

target_include_directories(ecmaster_demo PUBLIC
    ${ECM_SDK_ROOT}/INC
    ${ECM_SDK_ROOT}/INC/Linux
    ${ECM_SOURCE_ROOT}/Common
    ${ECM_SOURCE_ROOT}/LinkOsLayer
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/Common
    ${CMAKE_CURRENT_SOURCE_DIR}/Common/Linux
    ${CMAKE_CURRENT_SOURCE_DIR}/../driver
)

target_link_directories(ecmaster_demo PUBLIC ${ECM_SDK_LIB_DIR})

target_link_libraries(ecmaster_demo
    EcMaster
    EcMasterRasServer
    pthread
    m
    dl
    rt
)

# Full demo executable (main is kept out of the static lib to avoid duplicate symbols)
add_executable(EcMasterDemoDc Common/Linux/EcDemoMain.cpp)
target_link_libraries(EcMasterDemoDc
    ecmaster_demo
    EcMaster
    EcMasterRasServer
    pthread
    m
    dl
    rt
)
target_include_directories(EcMasterDemoDc PRIVATE
    ${ECM_SDK_ROOT}/INC
    ${ECM_SDK_ROOT}/INC/Linux
    ${ECM_SOURCE_ROOT}/Common
    ${ECM_SOURCE_ROOT}/LinkOsLayer
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/Common
    ${CMAKE_CURRENT_SOURCE_DIR}/Common/Linux
)
target_link_directories(EcMasterDemoDc PRIVATE ${ECM_SDK_LIB_DIR})
