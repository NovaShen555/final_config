//
// Created by 19963 on 2025/7/8.
//

#ifndef FILTER_H
#define FILTER_H


// 定义卡尔曼滤波器结构??
typedef struct {
    float Q;         // 过程噪声的方??
    float R;         // 测量噪声的方??
    float x_last;    // 上一时刻的状态估�???????
    float p_last;    // 上一时刻的估计误差协方差
} KalmanFilter;

void kalman_filter_init(KalmanFilter *filter, float Q, float R);
float kalman_filter_update(KalmanFilter *filter, float measurement);

#endif //FILTER_H
