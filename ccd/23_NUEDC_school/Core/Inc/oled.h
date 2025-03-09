#ifndef __OLED_H
#define __OLED_H

#include "stdlib.h"
#include "gpio.h"

//-----------------OLED�˿ڶ���----------------

#define OLED_SCL_Clr() LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_9) // SCL
#define OLED_SCL_Set() LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_9)

#define OLED_SDA_Clr() LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_10) // DIN
#define OLED_SDA_Set() LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_10)

// #define OLED_RES_Clr() LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_2)//RES
// #define OLED_RES_Set() LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_2)

#define OLED_CMD 0  
#define OLED_DATA 1 

//void OLED_ClearPoint(uint8_t x, uint8_t y);
void OLED_ColorTurn(uint8_t i);
void OLED_DisplayTurn(uint8_t i);
//void Send_Byte(uint8_t dat);
void OLED_WR_Byte(uint8_t dat, uint8_t mode);
void OLED_DisPlay_On(void);
void OLED_DisPlay_Off(void);
void OLED_Refresh(void);
void OLED_Clear(void);
void OLED_DrawPoint(uint8_t x, uint8_t y, uint8_t t);
void OLED_DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t mode);
void OLED_DrawCircle(uint8_t x, uint8_t y, uint8_t r);
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t size1, uint8_t mode);
void OLED_ShowChar6x8(uint8_t x, uint8_t y, uint8_t chr, uint8_t mode);
void OLED_ShowString(uint8_t x, uint8_t y, uint8_t *chr, uint8_t size1, uint8_t mode);
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size1, uint8_t mode);
void OLED_ShowChinese(uint8_t x, uint8_t y, uint8_t num, uint8_t size1, uint8_t mode);
void OLED_ScrollDisplay(uint8_t num, uint8_t space, uint8_t mode);
void OLED_ShowPicture(uint8_t x, uint8_t y, uint8_t sizex, uint8_t sizey, uint8_t BMP[], uint8_t mode);
void OLED_Init(void);
void OLED_ShowChart(uint16_t *data, uint16_t mode, uint16_t range, uint16_t threshold);

#endif
