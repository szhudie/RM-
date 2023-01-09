#ifndef __BSP_CAN_H__
#define __BSP_CAN_H__

#include "pid.h"
#include "struct_typedef.h"


extern void CAN_cmd_chassis(int16_t motor1, 
	                          int16_t motor2, 
                            int16_t motor3, 
                            int16_t motor4);


void can_filter_init(void);
void Can_init(void);
typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,

    CAN_YAW_MOTOR_ID = 0x205,
    CAN_PIT_MOTOR_ID = 0x206,
    CAN_TRIGGER_MOTOR_ID = 0x207,
    CAN_GIMBAL_ALL_ID = 0x1FF,

} can_msg_id_e;

typedef struct
{   uint16_t ecd_flag;
    int16_t ecd;
	  int32_t ecd_out;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
	int32_t angle_error;
	  int quan;
	
} motor_measure_t;

typedef struct
{
  const motor_measure_t *chassis_motor_measure;
  fp32 accel;
  fp32 speed;
  fp32 speed_set;
  int16_t give_current;
} chassis_motor_t;


extern const motor_measure_t *get_Chassis_Motor_Measure_Point(uint8_t i);
extern const motor_measure_t *get_Yaw_Gimbal_Motor_Measure_Point(void);





extern motor_measure_t motor_chassis[4];

void Can2_init(void);


#endif

