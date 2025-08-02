#include "gray.h"


unsigned char IIC_ReadByte(unsigned char Salve_Adress,int ordinal )
{
	unsigned char dat;
	if(ordinal == front)
		HAL_I2C_Master_Receive(&hi2c3,Salve_Adress<<1,&dat,1,1000);
	else if(ordinal == back)
		HAL_I2C_Master_Receive(&hi2c4,Salve_Adress<<1,&dat,1,1000);
	return dat;
}

unsigned char IIC_ReadBytes(unsigned char Salve_Adress,unsigned char Reg_Address,unsigned char *Result,unsigned char len,int ordinal)
{
	if(ordinal == front)
		return HAL_I2C_Mem_Read(&hi2c3,Salve_Adress,Reg_Address,I2C_MEMADD_SIZE_8BIT,Result,len,1000)==HAL_OK;
	else if(ordinal == back)
		return HAL_I2C_Mem_Read(&hi2c4,Salve_Adress,Reg_Address,I2C_MEMADD_SIZE_8BIT,Result,len,1000)==HAL_OK;
}

unsigned char IIC_WriteByte(unsigned char Salve_Adress,unsigned char Reg_Address,unsigned char data, int ordinal)
{
	unsigned char dat[2]={Reg_Address,data};
	if(ordinal == front)
		return HAL_I2C_Master_Transmit(&hi2c3,Salve_Adress,dat,2,1000)==HAL_OK;
	else if(ordinal == back)
		return HAL_I2C_Master_Transmit(&hi2c4,Salve_Adress,dat,2,1000)==HAL_OK;

}
unsigned char IIC_WriteBytes(unsigned char Salve_Adress,unsigned char Reg_Address,unsigned char *data,unsigned char len,int ordinal)
{
if(ordinal == front)
	return HAL_I2C_Mem_Write(&hi2c3,Salve_Adress,Reg_Address,I2C_MEMADD_SIZE_8BIT,data,len, 1000)==HAL_OK;
	else if(ordinal == back)
		return HAL_I2C_Mem_Write(&hi2c4,Salve_Adress,Reg_Address,I2C_MEMADD_SIZE_8BIT,data,len, 1000)==HAL_OK;
}

unsigned char Ping(void)
{
	unsigned char dat_f;
	unsigned char dat_b;
	IIC_ReadBytes(GW_GRAY_ADDR_DEF<<1,GW_GRAY_PING,&dat_b,1,back);
	IIC_ReadBytes(GW_GRAY_ADDR_DEF<<1,GW_GRAY_PING,&dat_f,1,front);
	if((dat_f==GW_GRAY_PING_OK)&&(dat_b==GW_GRAY_PING_OK))
	{
			return 0;
	}	
	else return 1;
}
unsigned char IIC_Get_Digtal(int ordinal)
{
	unsigned char dat;
IIC_ReadBytes(GW_GRAY_ADDR_DEF<<1,GW_GRAY_DIGITAL_MODE,&dat,1,ordinal);
	return dat;
}
unsigned char IIC_Get_Anolog(unsigned char * Result,unsigned char len,int ordinal)
{
		if(IIC_ReadBytes(GW_GRAY_ADDR_DEF<<1,GW_GRAY_ANALOG_BASE_,Result,len,ordinal))return 1;
	else return 0;
}
unsigned char IIC_Get_Single_Anolog(unsigned char Channel,int ordinal)
{
	unsigned char dat;
	IIC_ReadBytes(GW_GRAY_ADDR_DEF<<1,GW_GRAY_ANALOG(Channel),&dat,1,ordinal);
	return dat;
}
unsigned char IIC_Anolog_Normalize(uint8_t Normalize_channel,int ordinal )
{
	return IIC_WriteBytes(GW_GRAY_ADDR_DEF<<1,0xCF,&Normalize_channel,1,ordinal);
}
unsigned short IIC_Get_Offset(int ordinal )
{
	unsigned char dat[2]={0};
	IIC_ReadBytes(GW_GRAY_ADDR_DEF<<1,Offset,dat,2,ordinal);
	return (unsigned short)dat[0]|(unsigned short)dat[1]<<8;
}
