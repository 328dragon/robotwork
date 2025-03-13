#include "tcs230.h"


uint8_t USARTData;//数据缓冲
uint8_t USARTDataBag[6];//数据存储数组

uint8_t Get_RxFlag(void)
{
	if (RX_Flag == 1)
	{
		RX_Flag = 0;
		return 1;
	}
	return 0;
}
int Get_Color(void)
{
	if (R > 150 && G > 150 && B > 150)
	{

		// return 135;
		return 45;
	} // 白色
	  //  if(B-R>=20&&B-G>=20&&B>=60)
//	if (B - R >= 20 && B - G >= 0 && B >= 60)
	if((B > R) && (B > G) && (B > 1.7 * R))

	{

		// return 0;
		return 180;
	} // 蓝色
	if (G > B && G > R && G >= 20)
	//   if(G>R&&G>=20)

	{

		// return 180;
		return 0;
	} // 绿色
	if (R - B >= 40 && R - G >= 40 && R >= 80)
	{

		return 90;
	} // 红色
	if (R <= 50 && G <= 50 && B <= 50)
	{

		// return 45;
		return 135;
	} // 黑色

	return -1;
}

uint8_t Color_Recognize(void)
{
	R = 0;
	G = 0;
	B = 0;
	int color = -1;
	printf("AT+COLOR\r\n");
	HAL_Delay(500);
	printf("AT+COLOR\r\n");

	while (1)
	{
		//			BUZZER();
		if (Get_RxFlag() && R != 0 && G != 0 && B != 0)
		{
			color = Get_Color();

			//				return (Get_Color());

			if (color != -1)
				return color;	

			else
			{
			
			}
		}
		printf("AT+COLOR\r\n");
	}
}


