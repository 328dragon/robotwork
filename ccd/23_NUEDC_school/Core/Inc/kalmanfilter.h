/*
 * @Author: Ptisak
 * @Date: 2023-04-21 23:38:54
 * @LastEditors: Ptisak
 * @LastEditTime: 2023-05-19 11:41:42
 * @Version: Do not edit
 */
/*
 * D_zhili.h
 *
 *  Created on: 2022Äê3ÔÂ8ÈÕ
 *      Author: Breeze
 */

#ifndef CODE_D_ZHILI_H_
#define CODE_D_ZHILI_H_

#include "main.h"
typedef struct
{
    float p, q, r, kGain, prevData;
} KalmanFilter_t;

void KalmanFilter_init(KalmanFilter_t *kf, float q, float r);
float KalmanFilter(KalmanFilter_t *kf, float inData);
#endif /* CODE_D_ZHILI_H_ */
