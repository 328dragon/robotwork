#ifndef GRAY_H
#define GRAY_H
#include "stm32g4xx_hal.h"
#include "i2c.h"
#include "gw_grayscale_sensor.h"
enum gray_ordinal
{
front=0,
back=1,
};
unsigned char Ping(void);
unsigned char IIC_Get_Digtal(int ordinal);
unsigned char IIC_Get_Anolog(unsigned char * Result,unsigned char len,int ordinal);
unsigned char IIC_Get_Single_Anolog(unsigned char Channel,int ordinal);
unsigned char IIC_Anolog_Normalize(uint8_t Normalize_channel,int ordinal );
unsigned short IIC_Get_Offset(int ordinal );
// unsigned char Ping(void);
// unsigned char IIC_Get_Digtal(void);
// unsigned char IIC_Get_Anolog(unsigned char * Result,unsigned char len);
// unsigned char IIC_Get_Single_Anolog(unsigned char Channel);
// unsigned char IIC_Anolog_Normalize(uint8_t Normalize_channel);
// unsigned short IIC_Get_Offset(void );

#endif
