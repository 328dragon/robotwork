#include "hwt905.h"
#include <math.h>
#include  <stddef.h> 
#define M_PI 3.1415926
// 将原始数据转换为角度值
float getralval(uint8_t *data)
{
    int16_t val = (data[1] << 8) | data[0];
    
    // 将16位整数转换为弧度值，范围是-π到π
    return (val / 32768.0f) * M_PI;
}

// 校准Yaw角，设置零点
void setzeroyaw(IMU_t *imu)
{
    if (imu == NULL) return;
    imu->zero_yaw = getralval(imu->eular.yaw.data);
}

// 获取当前Yaw角（相对于零点）
float getyaw(IMU_t *imu)
{
    if (imu == NULL) return 0.0f;
    return getralval(imu->eular.yaw.data) - imu->zero_yaw;
}

// 更新IMU数据
void update(IMU_t *imu)
{
    if (imu == NULL) return;
    
    uint8_t *data = imu->buffer;
    if (data[0] == 0x55)
    {
        switch (data[1])
        {
            case 0x53:
                imu->eular.roll.data[0] = data[2];
                imu->eular.roll.data[1] = data[3];
                imu->eular.pitch.data[0] = data[4];
                imu->eular.pitch.data[1] = data[5];
                imu->eular.yaw.data[0] = data[6];
                imu->eular.yaw.data[1] = data[7];
                break;
            default:
                break;
        }
    }
}
