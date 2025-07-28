#include "control.h"

const float   kp = 12,
        ki = 0.1,
        kd = 1;

PID_LocTypeDef Motor_A_PID={
    kp,
    ki,
    kd,
    0,0,0,0};

PID_LocTypeDef Motor_B_PID={
    kp,
    ki,
    kd,
    0,0,0,0};


PID_LocTypeDef Z_PID={
    15,
    0.1,
    5,
    0,0,0,0};

// Created by link on 25-7-3.
void PID_output(float set,float ture, PID_LocTypeDef *PID, float limit){

    (PID->er)=set-ture;
    PID->LocSum=PID->LocSum+PID->er;
    PID->output = (PID->Kp)*(PID->er)+(PID->Ki)*(PID->LocSum)-(PID->Kd)*(PID->er-PID->er_1);
    if (PID->LocSum >= limit){

        PID->LocSum = limit;
    }

    if ( PID->LocSum <= -limit){

        PID->LocSum =-limit;

    }
    PID->er_1=PID->er;
}

void PID_output_2(float set,float ture, PID_LocTypeDef *PID, float limit){

    (PID->er)=set-ture;
    PID->LocSum=PID->LocSum+PID->er;
    PID->output = (PID->Kp)*(set)+(PID->Ki)*(PID->LocSum)-(PID->Kd)*(PID->er-PID->er_1);
    if (PID->LocSum >= limit){

        PID->LocSum = limit;
    }

    if ( PID->LocSum <= -limit){

        PID->LocSum =-limit;

    }
    PID->er_1=PID->er;
}
