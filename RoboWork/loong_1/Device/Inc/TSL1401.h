#ifndef __TSL1401_H
#define __TSL1401_H
#include "main.h"
#include "adc.h"
extern uint8_t Flag_Target;
extern uint8_t CCD_Zhongzhi,CCD_Yuzhi,Last_CCD_Zhongzhi;
extern uint16_t ADV[128]; 

void Dly_us(void);
uint16_t Get_Adc(uint8_t ch) ;
void RD_TSL(void);
void  Find_CCD_Zhongzhi(void);
int myabs(int a);


#define  clk_A_on HAL_GPIO_WritePin(CLK_A_GPIO_Port,CLK_A_Pin,1);
#define  clk_A_off HAL_GPIO_WritePin(CLK_A_GPIO_Port,CLK_A_Pin,0);
#define  si_a_on HAL_GPIO_WritePin(SI_A_GPIO_Port,SI_A_Pin,1);
#define  si_a_off HAL_GPIO_WritePin(SI_A_GPIO_Port,SI_A_Pin,0);

#define 	clk_B_on HAL_GPIO_WritePin(CLK_B_GPIO_Port,CLK_B_Pin,1)
#define 	clk_B_off HAL_GPIO_WritePin(CLK_B_GPIO_Port,CLK_B_Pin,0)
#define  si_b_on HAL_GPIO_WritePin(SI_B_GPIO_Port,SI_B_Pin,1);
#define  si_b_off HAL_GPIO_WritePin(SI_B_GPIO_Port,SI_B_Pin,0);

#endif
