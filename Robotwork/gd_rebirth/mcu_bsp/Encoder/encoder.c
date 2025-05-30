#include "encoder.h"

/*
*该库用于增量式和绝对式的编码器读取值

*/

void userEncoderInitBinding(inc_encoder_t *encoder)
{
    // HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
    // HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    // HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
    // HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
}

// !用户需要自行编写每个编码器的读值函数,并在下面的函数中绑定
// !该函数返回的编码器读之包括正负值
// int userEncoderUpdate_0(uint8_t Clear)
// {
//     if (Clear==0)
//     {
//         return (int16_t)htim1.Instance->CNT;
//     }
//     else
//     {
//         __HAL_TIM_SetCounter(&htim1, 0);
//     }
//     return 0;
// }

// int userEncoderUpdate_1(uint8_t Clear)
// {
//     if (Clear==0)
//     {
//         return (int16_t)htim4.Instance->CNT;
//     }
//     else
//     {
//         __HAL_TIM_SetCounter(&htim4, 0);
//     }
//     return 0;
// }

// int userEncoderUpdate_2(uint8_t Clear)
// {
//     if (Clear == 0)
//     {
//         return (int16_t)htim5.Instance->CNT;
//     }
//     else
//     {
//         __HAL_TIM_SetCounter(&htim5, 0);
//     }
//     return 0;
// }

// int userEncoderUpdate_3(uint8_t Clear)
// {
//     if (Clear == 0)
//     {
//         return (int16_t)htim8.Instance->CNT;
//     }
//     else
//     {
//         __HAL_TIM_SetCounter(&htim8, 0);
//     }
//     return 0;
// }

void userEncoderUpdateBinding(inc_encoder_t *encoder)
{
    // if (encoder->id == 0)
    // {
    //     encoder->encoder_update = userEncoderUpdate_0;
    // }
    // if (encoder->id == 1)
    // {
    //     encoder->encoder_update = userEncoderUpdate_1;
    // }
    // if (encoder->id == 2)
    // {
    //     encoder->encoder_update = userEncoderUpdate_2;
    // }
    // if (encoder->id == 3)
    // {
    //     encoder->encoder_update = userEncoderUpdate_3;
    // }
}

//******************************
// !用户最后可调用的接口

void IncEncoderInit(inc_encoder_t *encoder, uint8_t id, float to_real_coefficient)
{
    memset(encoder, 0, sizeof(inc_encoder_t));

    encoder->id = id;
    encoder->to_real_coefficient = to_real_coefficient;
    encoder->pulse_sum = 0;
    encoder->pulse_sum_real = 0;

    userEncoderInitBinding(encoder);
    userEncoderUpdateBinding(encoder);
}

void IncEncoderUpdate(inc_encoder_t *encoder)
{
    encoder->pulse = encoder->encoder_update(0);    //读取编码器的值
    encoder->encoder_update(1);                     //清除编码器的值
    encoder->pulse_real = encoder->pulse * encoder->to_real_coefficient;
    encoder->pulse_sum += encoder->pulse;
    encoder->pulse_sum_real += encoder->pulse * encoder->to_real_coefficient;
}
