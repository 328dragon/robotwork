#ifndef __TSL1401_H
#define __TSL1401_H
#include "main.h"
#include "adc.h"

void Dly_us(void);
extern uint16_t ccd_data[128];     // 储存平均后的CCD数据
extern uint16_t  CCD_Yuzhi;
extern uint16_t  CCD_Zhongzhi;
void  Find_CCD_Zhongzhi(void);
#endif
