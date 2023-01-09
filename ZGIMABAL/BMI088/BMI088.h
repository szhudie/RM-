#ifndef BMI088_H
#define BMI088_H
#include "main.h"
#include "stm32f4xx_hal.h"


#define MESSAGE_GYRO      				1



//#define BMI088_ACC_PWR_CONF 0x7C
//#define BMI088_ACC_PWR_SUSPEND_MODE 0x03
//#define BMI088_ACC_PWR_ACTIVE_MODE 0x00

//#define BMI088_ACC_PWR_CTRL 0x7D
//#define BMI088_ACC_ENABLE_ACC_OFF 0x00
//#define BMI088_ACC_ENABLE_ACC_ON 0x04



uint8_t BMI088_Init(void);

uint8_t bmi088_accel_init(void);
uint8_t bmi088_accel_self_test(void);

uint8_t bmi088_gyro_init(void);
uint8_t bmi088_gyro_self_test(void);




		
#endif
