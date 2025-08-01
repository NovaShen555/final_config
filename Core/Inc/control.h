
// Created by link on 25-7-3.

#ifndef __CONTROL_H
#define __CONTROL_H


#define PI 3.14159265
#include "main.h"


typedef struct
{
    float Kp;
    float Ki;
    float Kd;

    float er;                       //
    float er_1;                      //
    float LocSum;
    float output;
}PID_LocTypeDef;

extern PID_LocTypeDef PID_V;//速度
extern PID_LocTypeDef Motor_A_PID;//速度
extern PID_LocTypeDef Motor_B_PID;//速度
extern PID_LocTypeDef Z_PID;//速度
extern PID_LocTypeDef PTZ_Z_PID;//速度
void PID_output(float SetValue, float ActualValue, PID_LocTypeDef *PID,float Limit);
void PID_output_2(float SetValue, float ActualValue, PID_LocTypeDef *PID,float Limit);
#endif