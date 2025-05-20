#include "bsp_IR_i2c.h"
extern I2C_HandleTypeDef hi2c2;
 //八路灰度
 	char huidu[128];
uint8_t ir_x1,ir_x2,ir_x3,ir_x4,ir_x5,ir_x6,ir_x7,ir_x8;

//进入校准
void set_adjust_mode(uint8_t mode)
{
	HAL_I2C_Mem_Write(&hi2c2, (0x12<<1), 0x01, I2C_MEMADD_SIZE_8BIT, &mode, 1, 10);//1:进入校准  0:退出校准
}
	


//读取红外数据
static  void read_IRdata(uint8_t *buf)
{
	HAL_I2C_Mem_Read(&hi2c2, (0x12<<1), 0x30, I2C_MEMADD_SIZE_8BIT, buf, 1, 10);
}
void deal_IRdata(uint8_t *x1,uint8_t *x2,uint8_t *x3,uint8_t *x4,uint8_t *x5,uint8_t *x6,uint8_t *x7,uint8_t *x8)
{
	uint8_t IRbuf = 0xFF;
	read_IRdata(&IRbuf);
	
	*x1 = (IRbuf>>7)&0x01;
	*x2 = (IRbuf>>6)&0x01;
	*x3 = (IRbuf>>5)&0x01;
	*x4 = (IRbuf>>4)&0x01;
	*x5 = (IRbuf>>3)&0x01;
	*x6 = (IRbuf>>2)&0x01;
	*x7 = (IRbuf>>1)&0x01;
	*x8 = (IRbuf>>0)&0x01;
}


