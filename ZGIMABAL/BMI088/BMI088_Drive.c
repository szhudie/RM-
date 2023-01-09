#include "BMI088_Drive.h"
#include "spi.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "math.h"
#include "Get_BMI088_Data.h"
uint8_t BMI088_Read_Write_Byte(uint8_t txdata)
{
    uint8_t rx_data;
    HAL_SPI_TransmitReceive(&hspi1, &txdata, &rx_data, 1, 1000);
	
    return rx_data;
}


//加速度计单个字节写入
void BMI088_ACCEL_Write_Byte_Reg(uint8_t reg, uint8_t transmit_data)
{

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	
	BMI088_Read_Write_Byte(reg);										//寄存器地址
	BMI088_Read_Write_Byte(transmit_data);
	
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	
}

//加速度计单个字节读取
void BMI088_ACCEL_Read_Byte_Reg(uint8_t reg, uint8_t *return_data)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	
  BMI088_Read_Write_Byte(reg | 0x80);						//寄存器地址
	BMI088_Read_Write_Byte(0x55);
	
  *return_data = BMI088_Read_Write_Byte(0x55);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

//加速度计多个字节读取
void BMI088_ACCEL_Read_Muli_Reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	
	BMI088_Read_Write_Byte(reg | 0x80);						//寄存器地址
	BMI088_Read_Write_Byte(0x55);
	
	while (len != 0)
	{
		*buf = 	BMI088_Read_Write_Byte(0x55);
		buf++;
		len--;
	}
	
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

//陀螺仪单个字节写入
void BMI088_GYRO_Write_Byte_Reg(uint8_t reg, uint8_t transmit_data)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	
	BMI088_Read_Write_Byte(reg);										//寄存器地址
	BMI088_Read_Write_Byte(transmit_data);
	
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	
}

//陀螺仪单个字节读取
void BMI088_GYRO_Read_Byte_Reg(uint8_t reg, uint8_t *return_data)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	
  BMI088_Read_Write_Byte(reg | 0x80);						//寄存器地址
  *return_data = BMI088_Read_Write_Byte(0x55);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
}



//陀螺仪多个字节读取
void BMI088_GYRO_Read_Muli_Reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	
	BMI088_Read_Write_Byte(reg | 0x80);						//寄存器地址
	
	while (len != 0)
	{
		*buf = 	BMI088_Read_Write_Byte(0x55);
		buf++;
		len--;
	}
	
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
}


void BMI088_Delay_us(uint16_t us)
{
    uint32_t ticks = 0;
    uint32_t told = 0;
    uint32_t tnow = 0;
    uint32_t tcnt = 0;
    uint32_t reload = 0;
    reload = SysTick->LOAD;
    ticks = us * 168;
    told = SysTick->VAL;
    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)
        {
            if (tnow < told)
            {
                tcnt += told - tnow;
            }
            else
            {
                tcnt += reload - tnow + told;
            }
            told = tnow;
            if (tcnt >= ticks)
            {
                break;
            }
        }
    }
}




//校准陀螺仪
void calibrate_imu(int n)
{
	static GYRO_INFO 	*GYRO_DATA;
	static ACC_INFO *ACC_DATA;

	GYRO_DATA = GET_GYRO_DATA();
	ACC_DATA = GET_ACC_DATA();
  int64_t offset_gyro_x = 0 ,offset_gyro_y = 0, offset_gyro_z = 0;
	ACC_DATA->acc_ratio = 1;
	
    GYRO_DATA->v_z_offset = 0;
		GYRO_DATA->v_y_offset = 0;
		GYRO_DATA->v_x_offset = 0;
	
    GYRO_DATA->v_z = 0;
    GYRO_DATA->v_y = 0;
    GYRO_DATA->v_x = 0;
	
    double acc_ratio_offset = 0;

    for(int i = 0; i< n; i++)
    {
        vTaskDelay(2);
				///////////////////////////////
				///////////////////////
				if(i%30 == 0)HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_12);

				get_imu_data();
				
        offset_gyro_x += GYRO_DATA->get_speed_Vx;
        offset_gyro_y += GYRO_DATA->get_speed_Vy;
        offset_gyro_z += GYRO_DATA->get_speed_Vz;
        
        acc_ratio_offset += sqrt(ACC_DATA->acc_x * ACC_DATA->acc_x + ACC_DATA->acc_y * ACC_DATA->acc_y + ACC_DATA->acc_z * ACC_DATA->acc_z);
    }
    
    
    GYRO_DATA->v_x_offset = (float)offset_gyro_x/n;
    GYRO_DATA->v_y_offset = (float)offset_gyro_y/n;
    GYRO_DATA->v_z_offset = (float)offset_gyro_z/n;
		ACC_DATA->acc_ratio = acc_ratio_offset/n;
		HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12,0);
}

	uint8_t get_speed[8];
	uint8_t get_acc[6];
	uint8_t get_tmp[2];


//陀螺仪解算
 void get_imu_data()
{
	static GYRO_INFO 	*GYRO_DATA;
	static ACC_INFO *ACC_DATA;

	GYRO_DATA = GET_GYRO_DATA();
	ACC_DATA = GET_ACC_DATA();


	
	BMI088_GYRO_Read_Muli_Reg(BMI088_GYRO_CHIP_ID,get_speed, 8);	
	if (get_speed[0] == BMI088_GYRO_CHIP_ID_VALUE)
	{
		GYRO_DATA->get_speed_Vx = (int16_t)((get_speed[3]) << 8) | get_speed[2];
		GYRO_DATA->get_speed_Vy = (int16_t)((get_speed[5]) << 8) | get_speed[4];
		GYRO_DATA->get_speed_Vz = (int16_t)((get_speed[7]) << 8) | get_speed[6];
//		
		GYRO_DATA->v_x = ((float)GYRO_DATA->get_speed_Vx  -  GYRO_DATA->v_x_offset)* 2000 /32768/57.29578;
		GYRO_DATA->v_y = ((float)GYRO_DATA->get_speed_Vy  -  GYRO_DATA->v_y_offset)* 2000 /32768/57.29578;
		GYRO_DATA->v_z = ((float)GYRO_DATA->get_speed_Vz  -  GYRO_DATA->v_z_offset)* 2000 /32768/57.29578;
//		GYRO_DATA->v_x = ((float)GYRO_DATA->get_speed_Vx )* BMI088_GYRO_SEN;
//		GYRO_DATA->v_y = ((float)GYRO_DATA->get_speed_Vy )* BMI088_GYRO_SEN;
//		GYRO_DATA->v_z = ((float)GYRO_DATA->get_speed_Vz )* BMI088_GYRO_SEN;
	}

	
	BMI088_ACCEL_Read_Muli_Reg(BMI088_ACCEL_XOUT_L,get_acc, 6);
	
	ACC_DATA->get_ACC_x  = (int16_t)(((get_acc[1]) << 8) | get_acc[0]);
	ACC_DATA->get_ACC_y  = (int16_t)(((get_acc[3]) << 8) | get_acc[2]);
	ACC_DATA->get_ACC_z  = (int16_t)(((get_acc[5]) << 8) | get_acc[4]);
	
  ACC_DATA->acc_x = ((float)(ACC_DATA->get_ACC_x * 3.0f) / ACC_DATA->acc_ratio)/32768;
  ACC_DATA->acc_y = ((float)(ACC_DATA->get_ACC_y * 3.0f) / ACC_DATA->acc_ratio)/32768;
  ACC_DATA->acc_z = ((float)(ACC_DATA->get_ACC_z * 3.0f) / ACC_DATA->acc_ratio)/32768;
//  ACC_DATA->acc_x = (float)(ACC_DATA->get_ACC_x * BMI088_ACCEL_SEN);
//  ACC_DATA->acc_y = (float)(ACC_DATA->get_ACC_y * BMI088_ACCEL_SEN);
//  ACC_DATA->acc_z = (float)(ACC_DATA->get_ACC_z * BMI088_ACCEL_SEN);
	
	BMI088_ACCEL_Read_Muli_Reg(BMI088_TEMP_M,get_tmp, 2);
	
	ACC_DATA->get_temp=(((get_tmp[1] & 0xE0) >> 5 )|get_tmp[0] << 3);
	if(ACC_DATA->get_temp>1023)	ACC_DATA->get_temp	-=	2048;
		ACC_DATA->temperature = (float)(ACC_DATA->get_temp*0.125) + 23;
}



