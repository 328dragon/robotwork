#ifndef __TSL1401_H
#define __TSL1401_H
#include "main.h"
#include "adc.h"

#define max(a, b) ((a) >= (b) ? (a) : (b))
#define min(a, b) ((a) <= (b) ? (a) : (b))

void FindLines(uint8_t *l, uint8_t *r, uint16_t *data, uint16_t threshold, uint8_t *l_w, uint8_t *r_w);
int myabs(int a);

extern uint16_t ccd_rawdata[1546]; // 储存CCD接收的原始数据
extern uint16_t ccd_data[128];     // 储存平均后的CCD数据
extern float L_speed_setpoint, R_speed_setpoint;
extern uint32_t icg_flag, stop_line_flag;
extern uint8_t l, r;
extern uint8_t l_w, r_w;
extern uint16_t avg, _max, _min;
extern uint8_t inner_flag;



#endif
