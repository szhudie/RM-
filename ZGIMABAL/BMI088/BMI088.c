#include "BMI088.h"
#include "spi.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "cmsis_os.h"
#include "BMI088_Drive.h"

//static uint8_t write_BMI088_ACCEL_Reg_Data_Error[BMI088_Write_ACCEL_Reg_Num][3] =
//{
//    {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_Error},
//    {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_Error},
//    {BMI088_ACC_CONF,  BMI088_ACC_OSR4 | BMI088_ACC_50_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_Error},
//    {BMI088_ACC_RANGE, BMI088_ACC_RANGE_3G, BMI088_ACC_RANGE_Error},
//    {BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW, BMI088_INT1_IO_CTRL_Error},
//    {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_Error}

//};
//static uint8_t write_BMI088_GYRO_Reg_Data_Error[BMI088_Write_GYRO_Reg_Num][3] =
//{X1251WRS-02HF-LPSW
//{BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_Error},
//{BMI088_GYRO_BANDWIDTH, BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set, BMI088_GYRO_BANDWIDTH_Error},
//{BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_Error},
//{BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_Error},
//{BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW, BMI088_GYRO_INT3_INT4_IO_CONF_Error},
//{BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_Error}
//};

static uint8_t write_BMI088_ACCEL_Reg_Data_Error[BMI088_Write_ACCEL_Reg_Num][3] =
    {
        {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, 1},
        {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, 2},
        {BMI088_ACC_CONF,  BMI088_ACC_NORMAL| BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set, 3},
        {BMI088_ACC_RANGE, BMI088_ACC_RANGE_3G, 4},
        {BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW, 5},
        {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, 6}
};

static uint8_t write_BMI088_GYRO_Reg_Data_Error[BMI088_Write_GYRO_Reg_Num][3] =
{
        {BMI088_GYRO_RANGE, BMI088_GYRO_2000, 1},
        {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set, 2},
        {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, 3},
        {BMI088_GYRO_CTRL, BMI088_DRDY_ON, 4},
        {BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW, 5},
        {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, 6}
};



static 	uint8_t error = BMI088_NO_Error;

uint8_t BMI088_Init()
{
	//加速计

	if (bmi088_accel_self_test() != BMI088_NO_Error)
	{
			error |= BMI088_SELF_TEST_ACCEL_Error;
	}
	else
	{
			error |= bmi088_accel_init();
	}
	osDelay(1);
	//陀螺仪
	if (bmi088_gyro_self_test() != BMI088_NO_Error)
	{
			error |= BMI088_SELF_TEST_GYRO_Error;
	}
	else
	{
			error |= bmi088_gyro_init();
	}

	return error;
	
}




uint8_t bmi088_accel_init(void)
{
    uint8_t res = 0;
    uint8_t write_reg_num = 0;

    //check commiunication
    BMI088_ACCEL_Read_Byte_Reg(BMI088_ACC_CHIP_ID, &res);
    BMI088_Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_ACCEL_Read_Byte_Reg(BMI088_ACC_CHIP_ID, &res);
    BMI088_Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    //accel software reset
    BMI088_ACCEL_Write_Byte_Reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    osDelay(BMI088_LONG_DELAY_TIME);

    //check commiunication is normal after reset
    BMI088_ACCEL_Read_Byte_Reg(BMI088_ACC_CHIP_ID, &res);
    BMI088_Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_ACCEL_Read_Byte_Reg(BMI088_ACC_CHIP_ID, &res);
    BMI088_Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // check the "who am I"
    if (res != BMI088_ACC_CHIP_ID_VALUE)
    {
        return BMI088_NO_Sensor;
    }

    //set accel sonsor config and check
    for (write_reg_num = 0; write_reg_num < BMI088_Write_ACCEL_Reg_Num; write_reg_num++)
    {

        BMI088_ACCEL_Write_Byte_Reg(write_BMI088_ACCEL_Reg_Data_Error[write_reg_num][0], write_BMI088_ACCEL_Reg_Data_Error[write_reg_num][1]);
        BMI088_Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        BMI088_ACCEL_Read_Byte_Reg(write_BMI088_ACCEL_Reg_Data_Error[write_reg_num][0], &res);
        BMI088_Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != write_BMI088_ACCEL_Reg_Data_Error[write_reg_num][1])
        {
            return write_BMI088_ACCEL_Reg_Data_Error[write_reg_num][2];
        }
    }
    return BMI088_NO_Error;
}



static    int16_t self_test_accel[2][3];
static     uint8_t BMI088_ACCEL_buf[6] = {0, 0, 0, 0, 0, 0};

uint8_t bmi088_accel_self_test(void)
{

static uint8_t res = 0;

    uint8_t write_reg_num = 0;

    static const uint8_t write_BMI088_ACCEL_self_test_Reg_Data_Error[6][3] =
    {
        {BMI088_ACC_CONF, BMI088_ACC_NORMAL | BMI088_ACC_1600_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_Error},
        {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_Error},
        {BMI088_ACC_RANGE, BMI088_ACC_RANGE_24G, BMI088_ACC_RANGE_Error},
        {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_Error},
        {BMI088_ACC_SELF_TEST, BMI088_ACC_SELF_TEST_POSITIVE_SIGNAL, BMI088_ACC_PWR_CONF_Error},
        {BMI088_ACC_SELF_TEST, BMI088_ACC_SELF_TEST_NEGATIVE_SIGNAL, BMI088_ACC_PWR_CONF_Error}
    };

    //check commiunication is normal
    BMI088_ACCEL_Read_Byte_Reg(BMI088_ACC_CHIP_ID, &res);
    BMI088_Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_ACCEL_Read_Byte_Reg(BMI088_ACC_CHIP_ID, &res);
    BMI088_Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // reset  bmi088 accel sensor and wait for > 50ms
    BMI088_ACCEL_Write_Byte_Reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    osDelay(BMI088_LONG_DELAY_TIME);

    //check commiunication is normal
    BMI088_ACCEL_Read_Byte_Reg(BMI088_ACC_CHIP_ID, &res);
    BMI088_Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_ACCEL_Read_Byte_Reg(BMI088_ACC_CHIP_ID, &res);
    BMI088_Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    if (res != BMI088_ACC_CHIP_ID_VALUE)
    {
        return BMI088_NO_Sensor;
    }

    // set the accel register
    for (write_reg_num = 0; write_reg_num < 4; write_reg_num++)
    {

        BMI088_ACCEL_Write_Byte_Reg(write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num][0], write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num][1]);
        BMI088_Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        BMI088_ACCEL_Read_Byte_Reg(write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num][0], &res);
        BMI088_Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num][1])
        {
            return write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num][2];
        }
        // accel conf and accel range  . the two register set need wait for > 50ms
        osDelay(BMI088_LONG_DELAY_TIME);
    }
    // self test include postive and negative
    for (write_reg_num = 0; write_reg_num < 2; write_reg_num++)
    {
        BMI088_ACCEL_Write_Byte_Reg(write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num + 4][0], write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num + 4][1]);
        BMI088_Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        BMI088_ACCEL_Read_Byte_Reg(write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num + 4][0], &res);
        BMI088_Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num + 4][1])
        {
            return write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num + 4][2];
        }
        // accel conf and accel range  . the two register set need wait for > 50ms
        osDelay(BMI088_LONG_DELAY_TIME);																																//等待50ms
        // read response accel
        BMI088_ACCEL_Read_Muli_Reg(BMI088_ACCEL_XOUT_L, &BMI088_ACCEL_buf[0], 6);

        self_test_accel[write_reg_num][0] = (int16_t)(((BMI088_ACCEL_buf[1]) << 8) | BMI088_ACCEL_buf[0]);
        self_test_accel[write_reg_num][1] = (int16_t)(((BMI088_ACCEL_buf[3]) << 8) | BMI088_ACCEL_buf[2]);
        self_test_accel[write_reg_num][2] = (int16_t)(((BMI088_ACCEL_buf[5]) << 8) | BMI088_ACCEL_buf[4]);
    }

    //set self test off
    BMI088_ACCEL_Write_Byte_Reg(BMI088_ACC_SELF_TEST, BMI088_ACC_SELF_TEST_OFF);
    BMI088_Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_ACCEL_Read_Byte_Reg(BMI088_ACC_SELF_TEST, &res);
    BMI088_Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    if (res != (BMI088_ACC_SELF_TEST_OFF))
    {
        return BMI088_ACC_SELF_TEST_Error;
    }

    //reset the accel sensor
    BMI088_ACCEL_Write_Byte_Reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    osDelay(BMI088_LONG_DELAY_TIME);

    if ((self_test_accel[0][0] - self_test_accel[1][0] < 1365) || (self_test_accel[0][1] - self_test_accel[1][1] < 1365) || (self_test_accel[0][2] - self_test_accel[1][2] < 680))
    {
        return BMI088_SELF_TEST_ACCEL_Error;
    }

    BMI088_ACCEL_Read_Byte_Reg(BMI088_ACC_CHIP_ID, &res);
    BMI088_Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_ACCEL_Read_Byte_Reg(BMI088_ACC_CHIP_ID, &res);
    BMI088_Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    return BMI088_NO_Error;
}


uint8_t bmi088_gyro_init(void)
{
    uint8_t write_reg_num = 0;
    uint8_t res = 0;

    //check commiunication
    BMI088_GYRO_Read_Byte_Reg(BMI088_GYRO_CHIP_ID, &res);
    BMI088_Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_GYRO_Read_Byte_Reg(BMI088_GYRO_CHIP_ID, &res);
    BMI088_Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    //reset the gyro sensor
    BMI088_GYRO_Write_Byte_Reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    osDelay(BMI088_LONG_DELAY_TIME);
    //check commiunication is normal after reset
    BMI088_GYRO_Read_Byte_Reg(BMI088_GYRO_CHIP_ID, &res);
    BMI088_Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_GYRO_Read_Byte_Reg(BMI088_GYRO_CHIP_ID, &res);
    BMI088_Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // check the "who am I"
    if (res != BMI088_GYRO_CHIP_ID_VALUE)
    {
        return BMI088_NO_Sensor;
    }

    //set gyro sonsor config and check
    for (write_reg_num = 0; write_reg_num < BMI088_Write_GYRO_Reg_Num; write_reg_num++)
    {

        BMI088_GYRO_Write_Byte_Reg(write_BMI088_GYRO_Reg_Data_Error[write_reg_num][0], write_BMI088_GYRO_Reg_Data_Error[write_reg_num][1]);
        BMI088_Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        BMI088_GYRO_Read_Byte_Reg(write_BMI088_GYRO_Reg_Data_Error[write_reg_num][0], &res);
        BMI088_Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != write_BMI088_GYRO_Reg_Data_Error[write_reg_num][1])
        {
            return write_BMI088_GYRO_Reg_Data_Error[write_reg_num][2];
        }
    }

    return BMI088_NO_Error;
}

uint8_t bmi088_gyro_self_test(void)
{
    uint8_t res = 0;
    uint8_t retry = 0;
    //check commiunication is normal
    BMI088_GYRO_Read_Byte_Reg(BMI088_GYRO_CHIP_ID, &res);
    BMI088_Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_GYRO_Read_Byte_Reg(BMI088_GYRO_CHIP_ID, &res);
    BMI088_Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    //reset the gyro sensor
    BMI088_GYRO_Write_Byte_Reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    osDelay(BMI088_LONG_DELAY_TIME);
    //check commiunication is normal after reset
    BMI088_GYRO_Read_Byte_Reg(BMI088_GYRO_CHIP_ID, &res);
    BMI088_Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_GYRO_Read_Byte_Reg(BMI088_GYRO_CHIP_ID, &res);
    BMI088_Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    BMI088_GYRO_Write_Byte_Reg(BMI088_GYRO_SELF_TEST, BMI088_GYRO_TRIG_BIST);
    osDelay(BMI088_LONG_DELAY_TIME);

    do
    {
        BMI088_GYRO_Read_Byte_Reg(BMI088_GYRO_SELF_TEST, &res);
        BMI088_Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
        retry++;
    }
    while (!(res & BMI088_GYRO_BIST_RDY) && retry < 10);

    if (retry == 10)
    {
        return BMI088_SELF_TEST_GYRO_Error;
    }

    if (res & BMI088_GYRO_BIST_FAIL)
    {
        return BMI088_SELF_TEST_GYRO_Error;
    }

    return BMI088_NO_Error;
}

