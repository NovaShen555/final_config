//
// Created by 10415 on 25-7-26.
//

#include "PTZ.h"

#include <control.h>

void MY_Delay(float ms) {
    /* 1. 使能 DWT 和 CYCCNT（只需一次） */
    static uint8_t dwt_ok = 0;
    if (!dwt_ok) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; /* 使能 DWT 模块 */
        DWT->CYCCNT = 0;                                /* 清计数器 */
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;            /* 启动 CYCCNT */
        dwt_ok = 1;
    }

    /* 2. 计算目标 tick（480 MHz -> 480000 ticks/ms） */
    uint32_t ticks_per_ms = SystemCoreClock / 1000U;    /* = 480000 */
    uint32_t start = DWT->CYCCNT;
    uint32_t delay = (uint32_t)(ms * ticks_per_ms);

    /* 3. 阻塞等待（可中断） */
    while ((DWT->CYCCNT - start) < delay) {
        /* 空循环，编译器不会优化掉 */
        __NOP();
    }
}

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
    back_zero[3] = 0x52;
    back_zero[2] = 0x01;
    cr16(back_zero, 0, 5);
    HAL_UART_Transmit(&huart2, (uint8_t*)back_zero, sizeof(back_zero), HAL_MAX_DELAY);
    back_zero[2] = 0x02;
    cr16(back_zero, 0, 5);
    HAL_UART_Transmit(&huart2, (uint8_t*)back_zero, sizeof(back_zero), HAL_MAX_DELAY);
}

void PTZ_set_zero() {
    back_zero[3] = 0x21;
    back_zero[2] = 0x01;
    cr16(back_zero, 0, 5);
    HAL_UART_Transmit(&huart2, (uint8_t*)back_zero, sizeof(back_zero), HAL_MAX_DELAY);
    back_zero[2] = 0x02;
    MY_Delay(30);
    cr16(back_zero, 0, 5);
    HAL_UART_Transmit(&huart2, (uint8_t*)back_zero, sizeof(back_zero), HAL_MAX_DELAY);
    back_zero[3] = 0x52;
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


void PTZ_move_angle(char id, float angle) {
    // angle_move[1] = id - 1;
    angle_move[1] = 0x00;
    angle_move[2] = id;
    int16_t pulse = angle / PI * 8192;
    angle_move[5] = (pulse & 0xFF); // 低字节
    angle_move[6] = (pulse >> 8) & 0xFF; // 第二字节
    cr16(angle_move, 0, 7);
    HAL_UART_Transmit(&huart2, (uint8_t*)angle_move, sizeof(angle_move), HAL_MAX_DELAY);
}

void PTZ_update(float angle_x, float angle_z) {
    PTZ_set_angle(0x01, angle_x);
    MY_Delay(4);
    PTZ_set_angle(0x02, angle_z);
    // MY_Delay(4);
}

void PTZ_move(float angle_x, float angle_z) {
    PTZ_move_angle(0x01, angle_x);
    MY_Delay(3);
    PTZ_move_angle(0x02, angle_z);
    // MY_Delay(0);
}

char soft[] = {0x3e, 0x00, 0x01, 0x50, 0x00, 0x00, 0x00};
//                          id               cr16
void PTZ_soft() {
    soft[2] = 0x01;
    cr16(soft, 0, 5);
    HAL_UART_Transmit(&huart2, (uint8_t*)soft, sizeof(soft), HAL_MAX_DELAY);
    MY_Delay(10);
    soft[2] = 0x02;
    cr16(soft, 0, 5);
    HAL_UART_Transmit(&huart2, (uint8_t*)soft, sizeof(soft), HAL_MAX_DELAY);
}

char heartbeat[] = {0x3e, 0x00, 0x01, 0x0a, 0x00, 0x00, 0x00};
int PTZ_heartbeat() {
    cr16(heartbeat, 0, 5);
    HAL_UART_Transmit(&huart2, (uint8_t*)heartbeat, sizeof(heartbeat), HAL_MAX_DELAY);
    uint8_t rx_byte[] = {0xff, 0xff};

    HAL_UART_Receive(&huart2, rx_byte, 2, 15);

    if(rx_byte[0] != 0xff || rx_byte[1] != 0xff) return 1;

    return 0;
}

char getangle[] = {0x3e, 0x00, 0x01, 0x2f, 0x00, 0x00, 0x00};
float PTZ_getangle(int id) {
    getangle[2] = id;
    cr16(heartbeat, 0, 5);
    HAL_UART_Transmit(&huart2, (uint8_t*)heartbeat, sizeof(heartbeat), HAL_MAX_DELAY);
    uint8_t rx_byte[15];

    HAL_UART_Receive(&huart2, rx_byte, 15, 10);

    uint16_t data = rx_byte[6] <<8 | rx_byte[5];
    return data * 360 / 16384;
}
