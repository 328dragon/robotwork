#ifndef __BSP_IR_I2C_H_
#define __BSP_IR_I2C_H_


#include "main.h"
#define uint8_t uint8_t

#define IR_ADDRESS 0x12
void deal_IRdata(uint8_t *x1,uint8_t *x2,uint8_t *x3,uint8_t *x4,uint8_t *x5,uint8_t *x6,uint8_t *x7,uint8_t *x8);
void set_adjust_mode(uint8_t mode);
extern uint8_t ir_x1,ir_x2,ir_x3,ir_x4,ir_x5,ir_x6,ir_x7,ir_x8;
extern	char huidu[128];	

#endif

