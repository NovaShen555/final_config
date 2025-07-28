//
// Created by 10415 on 25-7-26.
//

#ifndef PTZ_H
#define PTZ_H

#include "usart.h"

static char back_zero[] = {0x3e, 0x00, 0x01, 0x51, 0x00, 0x00, 0x00};
//                                    电机id              cr16校验
static char angle_set[] = {0x3e, 0x00, 0x01, 0x55, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
//                                    电机id              脉冲4字节，小在前          cr16校验

void cr16(char *data, int begin, int end);// end指向最后一个数据之后

void PTZ_back_zero();

void PTZ_set_angle(char id, float angle);

void PTZ_update(float angle_x, float angle_z);

#endif //PTZ_H
