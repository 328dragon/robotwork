/*
 * @Author: Ptisak
 * @Date: 2023-04-21 23:38:54
 * @LastEditors: Ptisak
 * @LastEditTime: 2023-05-19 11:41:40
 * @Version: Do not edit
 */
#include <kalmanfilter.h>

void KalmanFilter_init(KalmanFilter_t *kf, float q, float r)
{
    kf->p = 10;
    kf->q = q;
    kf->r = r;
    kf->kGain = 0;
}
float KalmanFilter(KalmanFilter_t *kf, float inData)
{
    // static float prevData = 15;                       //上一个数据
    // static float p = 10, q = 0.1, r = 100, kGain = 0; // q 控制误差 r 控制响应速度
    if (!kf->prevData)
        kf->prevData = inData;
    kf->p = kf->p + kf->q;
    kf->kGain = kf->p / (kf->p + kf->r);                           // 计算卡尔曼增益
    inData = kf->prevData + (kf->kGain * (inData - kf->prevData)); // 计算本次滤波估计值
    kf->p = (1.f - kf->kGain) * kf->p;                             // 更新测量方差
    kf->prevData = inData;
    return inData; // 返回估计值
}
