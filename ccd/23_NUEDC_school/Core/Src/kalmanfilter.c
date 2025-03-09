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
    // static float prevData = 15;                       //��һ������
    // static float p = 10, q = 0.1, r = 100, kGain = 0; // q ������� r ������Ӧ�ٶ�
    if (!kf->prevData)
        kf->prevData = inData;
    kf->p = kf->p + kf->q;
    kf->kGain = kf->p / (kf->p + kf->r);                           // ���㿨��������
    inData = kf->prevData + (kf->kGain * (inData - kf->prevData)); // ���㱾���˲�����ֵ
    kf->p = (1.f - kf->kGain) * kf->p;                             // ���²�������
    kf->prevData = inData;
    return inData; // ���ع���ֵ
}
