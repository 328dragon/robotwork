#ifndef ENCODER_H_
#define ENCODER_H_

#include "main.h"
#include "string.h"
#include "tim.h"

typedef int (*IncEncoderUpdateCallback)(uint8_t clear);

typedef struct inc_encoder_t
{
    uint8_t id;

    int32_t pulse; //>以 count/s为单位
    int32_t pulse_sum;

    float pulse_real; //>以 mm/s 为单位
    float pulse_sum_real;

    float to_real_coefficient; //>该编码器对于实际转动的系数,根据读取的时间,编码器参数,轮子等参数决定
    float update_tick;         //>该编码器读值的更新时间

    // !非可调用接口
    IncEncoderUpdateCallback encoder_update;
} inc_encoder_t;

#endif

void IncEncoderInit(inc_encoder_t *encoder, uint8_t id, float to_real_coefficient);
void IncEncoderUpdate(inc_encoder_t *encoder);

