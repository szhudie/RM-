#ifndef __BSP_CAN_H__
#define __BSP_CAN_H__

#include "pid.h"
#include "struct_typedef.h"

void can_filter_init(void);
void Can_init(void);
typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
		CAN_FRI_MOTOR_ID1 = 0x201,
	  CAN_FRI_MOTOR_ID2 = 0x202,
	  CAN_TRI_MOTOR_ID = 0x203,//拨盘，摩擦轮电机ID

    CAN_YAW_MOTOR_ID = 0x205,
    CAN_PIT_MOTOR_ID = 0x206,
    CAN_TRIGGER_MOTOR_ID = 0x207,
    CAN_GIMBAL_ALL_ID = 0x1FF,

} can_msg_id_e;

typedef struct
{   uint16_t ecd_flag;
    int16_t ecd;//编码器不经处理的原始值
	  int32_t ecd_out;
    int16_t speed_rpm;//通过编码器计算得到的转速原始值
    int16_t given_current; //通过编码器计算得到的电机电流原始值
    uint8_t temperate;
    int16_t last_ecd;//上一次的编码器不经处理的原始值
	  int32_t angle_error;
	  float  total_angle;                 //电机角度
		float  angular_velocity;            //角速度
	  int quan;
	
} motor_measure_t;

//typedef struct
//{
//  const motor_measure_t *chassis_motor_measure;
//  fp32 accel;
//  fp32 speed;
//  fp32 speed_set;
//  int16_t give_current;
//} chassis_motor_t;
//发送云台控制命令，其中rev为保留字节
extern void CAN_cmd_gimbal(int16_t motor1,int16_t motor2,int16_t motor3, int16_t motor4);
//发送底盘电机控制命令
extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void send_shoot_cur(int16_t Fri1,int16_t Fri2,int16_t Tri1);
//返回yaw电机变量地址，通过指针方式获取原始数据
extern const motor_measure_t *get_Yaw_Gimbal_Motor_Measure_Point(void);
//返回pitch电机变量地址，通过指针方式获取原始数据
extern const motor_measure_t *get_Pitch_Gimbal_Motor_Measure_Point(void);
//返回trigger电机变量地址，通过指针方式获取原始数据
extern const motor_measure_t *get_Trigger_Motor_Measure_Point(void);
//返回底盘电机变量地址，通过指针方式获取原始数据,i的范围是0-3，对应0x201-0x204,
extern const motor_measure_t *get_Chassis_Motor_Measure_Point(uint8_t i);
extern const motor_measure_t *Get_Pluck_Motor_Data_Point(void);
const motor_measure_t *Get_Fri1_Motor_Data_Point(void);
const motor_measure_t *Get_Fri2_Motor_Data_Point(void);
extern motor_measure_t motor_chassis[7];
void Can2_init(void);
void CAN2_motor(uint8_t rx_data[8]);
void CAN2_get_remote(uint8_t *rx_data);
#endif

