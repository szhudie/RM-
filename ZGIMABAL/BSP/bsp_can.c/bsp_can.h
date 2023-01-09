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
	  CAN_TRI_MOTOR_ID = 0x203,//���̣�Ħ���ֵ��ID

    CAN_YAW_MOTOR_ID = 0x205,
    CAN_PIT_MOTOR_ID = 0x206,
    CAN_TRIGGER_MOTOR_ID = 0x207,
    CAN_GIMBAL_ALL_ID = 0x1FF,

} can_msg_id_e;

typedef struct
{   uint16_t ecd_flag;
    int16_t ecd;//���������������ԭʼֵ
	  int32_t ecd_out;
    int16_t speed_rpm;//ͨ������������õ���ת��ԭʼֵ
    int16_t given_current; //ͨ������������õ��ĵ������ԭʼֵ
    uint8_t temperate;
    int16_t last_ecd;//��һ�εı��������������ԭʼֵ
	  int32_t angle_error;
	  float  total_angle;                 //����Ƕ�
		float  angular_velocity;            //���ٶ�
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
//������̨�����������revΪ�����ֽ�
extern void CAN_cmd_gimbal(int16_t motor1,int16_t motor2,int16_t motor3, int16_t motor4);
//���͵��̵����������
extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void send_shoot_cur(int16_t Fri1,int16_t Fri2,int16_t Tri1);
//����yaw���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
extern const motor_measure_t *get_Yaw_Gimbal_Motor_Measure_Point(void);
//����pitch���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
extern const motor_measure_t *get_Pitch_Gimbal_Motor_Measure_Point(void);
//����trigger���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
extern const motor_measure_t *get_Trigger_Motor_Measure_Point(void);
//���ص��̵��������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����,i�ķ�Χ��0-3����Ӧ0x201-0x204,
extern const motor_measure_t *get_Chassis_Motor_Measure_Point(uint8_t i);
extern const motor_measure_t *Get_Pluck_Motor_Data_Point(void);
const motor_measure_t *Get_Fri1_Motor_Data_Point(void);
const motor_measure_t *Get_Fri2_Motor_Data_Point(void);
extern motor_measure_t motor_chassis[7];
void Can2_init(void);
void CAN2_motor(uint8_t rx_data[8]);
void CAN2_get_remote(uint8_t *rx_data);
#endif

