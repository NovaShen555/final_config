//
// Created by 10415 on 25-7-26.
//

#include "PTZ.h"

#include <control.h>

uint16_t crc16_modbus(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    while (len--) {
        crc ^= *data++;
        for (uint8_t i = 0; i < 8; i++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    /* 交换高低字节并返回 */
    return crc;
}

void cr16(char *data, int begin, int end) {
    uint16_t crc = crc16_modbus((const uint8_t *)(data + begin), end - begin);
    data[end] = crc & 0xFF; // 低字节
    data[end + 1] = (crc >> 8) & 0xFF; // 高字节
}

void PTZ_back_zero() {
    back_zero[2] = 0x01;
    cr16(back_zero, 0, 5);
    HAL_UART_Transmit(&huart2, (uint8_t*)back_zero, sizeof(back_zero), HAL_MAX_DELAY);
    back_zero[2] = 0x02;
    cr16(back_zero, 0, 5);
    HAL_UART_Transmit(&huart2, (uint8_t*)back_zero, sizeof(back_zero), HAL_MAX_DELAY);
}

void PTZ_set_angle(char id, float angle) {
    angle_set[1] = id - 1;
    angle_set[2] = id;
    int32_t pulse = angle / PI * 8192;
    angle_set[5] = (pulse & 0xFF); // 低字节
    angle_set[6] = (pulse >> 8) & 0xFF; // 第二字节
    angle_set[7] = (pulse >> 16) & 0xFF; // 第三字节
    angle_set[8] = (pulse >> 24) & 0xFF; // 高字节
    cr16(angle_set, 0, 9);
    HAL_UART_Transmit(&huart2, (uint8_t*)angle_set, sizeof(angle_set), HAL_MAX_DELAY);
}

void PTZ_update(float angle_x, float angle_z) {
    PTZ_set_angle(0x01, angle_x);
    HAL_Delay(2);
    PTZ_set_angle(0x02, angle_z);
}
