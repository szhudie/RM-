#include "IST8310.h"
#include "main.h"

extern I2C_HandleTypeDef hi2c3;

void IST8310_IIC_Init()
{
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_SET);
	HAL_Delay(100);	
}
void IST8310_IIC_WHO_AM_I()
{
	uint8_t data;

	while(data!=0x10)//0x10
	{
		IST8310_IIC_Init();
		HAL_I2C_Mem_Read(&hi2c3,READ_IST8310,0x00,I2C_MEMADD_SIZE_8BIT,&data,1, 10);
	}
	
	data = 0x08;
	while(HAL_I2C_Mem_Write(&hi2c3,WRITE_IST8310,0x0B,I2C_MEMADD_SIZE_8BIT,&data,1, 10)!=HAL_OK);//中断低电平
	
	data = 0x12;
	while(HAL_I2C_Mem_Write(&hi2c3,WRITE_IST8310,0x41,I2C_MEMADD_SIZE_8BIT,&data,1, 10)!=HAL_OK);//4次采样
	
	data = 0xC0;
	while(HAL_I2C_Mem_Write(&hi2c3,WRITE_IST8310,0x42,I2C_MEMADD_SIZE_8BIT,&data,1, 10)!=HAL_OK);//？

	data = 0x03;
	while(HAL_I2C_Mem_Write(&hi2c3,WRITE_IST8310,0x0A,I2C_MEMADD_SIZE_8BIT,&data,1, 10)!=HAL_OK);//200HZ输出
}


