#ifndef __TSL1401_H
#define __TSL1401_H
#include "main.h"
#include "adc.h"

void Dly_us(void);
uint16_t Get_Adc(uint8_t ch) ;
extern uint8_t Flag_Target;
extern uint8_t CCD_Zhongzhi,CCD_Yuzhi;
extern int ADV[128];
#endif
