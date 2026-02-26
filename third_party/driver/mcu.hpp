#pragma once

#include <cstdint>
#include <cstddef>

namespace basic_service {
namespace mcu {

/*
 * 网口帧格式（协议 - MCU）：
 * | 帧头 | 长度 | 类型 | CAN通道 | CAN_ID | 收发标志 | CAN帧类型 | DLC | DATA | 校验 |
 * | 2B   | 2B   | 1B   | 1B      | 2B     | 1B       | 1B        | 1B  | 8B   | 1B   |
 *
 * DATA（CAN 数据）：
 * | 发送命令字 | 对象索引 | 子对象索引 | 数据 |
 * | 1B        | 2B      | 1B        | 4B   |
 *
 * 帧头：0x55 0xAA
 * 长度：大端，数据段字节数（CAN通道 ~ DATA = 14）
 * 类型：0x02 = CAN
 * CAN_ID：大端，0x600 + 从站 ID
 * DLC：CAN 数据长度，固定 8
 * 校验：CRC8
 */

constexpr uint8_t FRAME_HEAD_0 = 0xAA;
constexpr uint8_t FRAME_HEAD_1 = 0x55;
constexpr uint8_t FRAME_TYPE_CAN = 0x02;
constexpr uint8_t FLAG_SEND = 0x01;
constexpr uint8_t FLAG_RECV = 0x00;
constexpr uint8_t CAN_FRAME_STD = 0x01;

// DATA 段：命令字(1) + 对象索引(2) + 子索引(1) + 数据(4) = 8
constexpr unsigned CAN_DATA_LEN = 8;

// 数据段：CAN通道(1) + CAN_ID(2) + 收发(1) + 帧类型(1) + DLC(1) + DATA(8) = 14
constexpr unsigned SEGMENT_LEN = 1 + 2 + 1 + 1 + 1 + CAN_DATA_LEN;

// 整帧：帧头(2) + 长度(2) + 类型(1) + 数据段(14) + 校验(1) = 20
constexpr unsigned FRAME_LEN = 2 + 2 + 1 + SEGMENT_LEN + 1;

bool CanSend(uint8_t channel, uint16_t can_id,
             uint8_t command, uint16_t object_index, uint8_t sub_index,
             const uint8_t data[4]);

bool CanRecv(uint16_t* can_id_out, uint8_t* command_out,
             uint16_t* obj_index_out, uint8_t* sub_index_out,
             uint8_t data_out[4], int timeout_ms);

bool Init(const char* host = nullptr, int port = 0);
void Shutdown();

}  // namespace mcu
}  // namespace basic_service
