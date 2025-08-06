#ifndef __HWT905_H
#define __HWT905_H

#include <stdint.h>

// 联合体定义，用于存储IMU数据
typedef union {
    uint8_t data[2];
} imu_data_t;

// 结构体定义，存储欧拉角数据
typedef struct {
    imu_data_t roll;
    imu_data_t pitch;
    imu_data_t yaw;
} ImuEular_t;

// IMU设备结构体，模拟C++中的类
typedef struct {
    uint8_t buffer[100];
    ImuEular_t eular;
    float zero_yaw;
} IMU_t;

// 函数声明，模拟类的成员函数
float getralval(uint8_t *data);
void setzeroyaw(IMU_t *imu);
float getyaw(IMU_t *imu);
void update(IMU_t *imu);
#endif