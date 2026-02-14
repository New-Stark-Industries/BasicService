#include "mcu.hpp"
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#if defined(__linux__) || defined(__APPLE__)
#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#define INVALID_SOCKET (-1)
typedef int sockfd_t;
#else
#error "Unsupported platform for MCU socket"
#endif

namespace basic_service
{
namespace mcu
{

namespace
{

sockfd_t g_sock = INVALID_SOCKET;

const char* g_mcu_ip = "192.168.114.123";
int g_mcu_port = 5000; // HOST->MCU
const char* g_local_ip = "192.168.114.124";
int g_local_port = 5001; // MCU->HOST

struct sockaddr_in g_mcu_addr = {};

static uint8_t Crc8(const uint8_t* data, size_t len)
{
    uint8_t crc = 0;
    while (len--)
    {
        crc ^= *data++;
        for (int i = 0; i < 8; i++)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
    return crc;
}

} // namespace

bool Init(const char* host, int port)
{
    if (g_sock != INVALID_SOCKET)
    {
        close(g_sock);
        g_sock = INVALID_SOCKET;
    }

    const char* mcu_ip = (host && host[0]) ? host : g_mcu_ip;
    int mcu_port = (port > 0) ? port : g_mcu_port;

    sockfd_t s = socket(AF_INET, SOCK_DGRAM, 0);
    if (s == INVALID_SOCKET)
    {
        perror("[MCU] socket 创建失败");
        return false;
    }

    struct sockaddr_in local_addr = {};
    local_addr.sin_family = AF_INET;
    local_addr.sin_port = htons((uint16_t)g_local_port);
    if (inet_pton(AF_INET, g_local_ip, &local_addr.sin_addr) <= 0)
    {
        close(s);
        return false;
    }
    if (bind(s, (struct sockaddr*)&local_addr, sizeof(local_addr)) < 0)
    {
        perror("[MCU] bind 失败");
        close(s);
        return false;
    }

    g_mcu_addr = {};
    g_mcu_addr.sin_family = AF_INET;
    g_mcu_addr.sin_port = htons((uint16_t)mcu_port);
    if (inet_pton(AF_INET, mcu_ip, &g_mcu_addr.sin_addr) <= 0)
    {
        close(s);
        return false;
    }

    g_sock = s;
    printf("[MCU] UDP 初始化成功: 发送 %s:%d, 接收 %s:%d\n",
           mcu_ip,
           mcu_port,
           g_local_ip,
           g_local_port);
    return true;
}

void Shutdown()
{
    if (g_sock != INVALID_SOCKET)
    {
        close(g_sock);
        g_sock = INVALID_SOCKET;
    }
}

/*
 * 组帧（20 字节）：
 * [0-1]   帧头       55 AA
 * [2-3]   长度       00 0E (14, 大端)
 * [4]     类型       02
 * [5]     CAN通道
 * [6-7]   CAN_ID     (大端)
 * [8]     收发标志
 * [9]     CAN帧类型
 * [10]    DLC        08
 * --- DATA (8B) ---
 * [11]    发送命令字
 * [12-13] 对象索引   (小端)
 * [14]    子对象索引
 * [15-18] 数据       (4B)
 * ---
 * [19]    CRC8
 */
bool CanSend(uint8_t channel,
             uint16_t can_id,
             uint8_t command,
             uint16_t object_index,
             uint8_t sub_index,
             const uint8_t data[4])
{
    if (g_sock == INVALID_SOCKET)
        return false;

    uint8_t buf[FRAME_LEN];
    size_t off = 0;

    // 帧头
    buf[off++] = FRAME_HEAD_0;
    buf[off++] = FRAME_HEAD_1;
    // 长度（大端）
    buf[off++] = (uint8_t)((SEGMENT_LEN >> 8) & 0xFF);
    buf[off++] = (uint8_t)(SEGMENT_LEN & 0xFF);
    // 类型
    buf[off++] = FRAME_TYPE_CAN;
    // CAN 通道
    buf[off++] = channel;
    // CAN_ID 大端
    buf[off++] = (uint8_t)((can_id >> 8) & 0xFF);
    buf[off++] = (uint8_t)(can_id & 0xFF);
    // 收发标志
    buf[off++] = FLAG_SEND;
    // CAN 帧类型
    buf[off++] = CAN_FRAME_STD;
    // DLC
    buf[off++] = 0x08;
    // --- DATA ---
    buf[off++] = command;
    buf[off++] = (uint8_t)(object_index & 0xFF);
    buf[off++] = (uint8_t)((object_index >> 8) & 0xFF);
    buf[off++] = sub_index;
    if (data)
    {
        memcpy(buf + off, data, 4);
    }
    else
    {
        memset(buf + off, 0, 4);
    }
    off += 4;

    // CRC8（从长度字段到数据段末）
    uint8_t crc = Crc8(buf + 2, (size_t)(off - 2));
    buf[off++] = crc;

    ssize_t n = sendto(g_sock, buf, off, 0, (struct sockaddr*)&g_mcu_addr, sizeof(g_mcu_addr));
    return (n == (ssize_t)off);
}

/*
 * 解析响应帧，与发送帧格式一致（固定偏移）：
 * [0-1]   帧头       AA 55
 * [2-3]   长度       大端
 * [4]     类型       02
 * [5]     CAN通道
 * [6-7]   CAN_ID     大端
 * [8]     收发标志
 * [9]     CAN帧类型
 * [10]    DLC
 * [11]    发送命令字
 * [12-13] 对象索引   小端
 * [14]    子对象索引
 * [15-18] 数据       4B
 * [19]    CRC8
 */
bool CanRecv(uint16_t* can_id_out,
             uint8_t* command_out,
             uint16_t* obj_index_out,
             uint8_t* sub_index_out,
             uint8_t data_out[4],
             int timeout_ms)
{
    if (g_sock == INVALID_SOCKET)
        return false;

    struct pollfd pfd = { g_sock, POLLIN, 0 };
    int r = poll(&pfd, 1, timeout_ms);
    if (r <= 0)
        return false;

    uint8_t buf[64];
    ssize_t n = recvfrom(g_sock, buf, sizeof(buf), 0, nullptr, nullptr);
    if (n < (ssize_t)FRAME_LEN)
        return false;

    // 校验帧头
    if (buf[0] != FRAME_HEAD_0 || buf[1] != FRAME_HEAD_1)
        return false;

    // CAN_ID 大端 [6-7]
    if (can_id_out)
        *can_id_out = (uint16_t)((buf[6] << 8) | buf[7]);
    // DATA 固定偏移 [11-18]
    if (command_out)
        *command_out = buf[11];
    if (obj_index_out)
        *obj_index_out = (uint16_t)(buf[12] | (buf[13] << 8));
    if (sub_index_out)
        *sub_index_out = buf[14];
    if (data_out)
        memcpy(data_out, buf + 15, 4);

    return true;
}

} // namespace mcu
} // namespace basic_service
