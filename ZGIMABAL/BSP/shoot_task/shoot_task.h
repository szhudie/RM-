#ifndef SHOOT_TASK_H
#define SHOOT_TASK_H

#include "main.h"
#include "pid.h"
#include "Gimbal_Task.h"

#define Bullet_ON 1600
#define Bullet_OFF 4000

#define BULLET_DEADLINE 10
#define Shoot_MAXIOUT 4000
#define Shoot_MAXOUT 8000
#define Fri_MAXOUT 6000

#define UP_BULLET_DEADLINE 400
#define BULLET_DEADLINE1 10
#define Down_BULLET_DEADLINE -400

#define KEEP_SHOOT_TIME1 200 //�ж������ǰҡʱ��(ң����)
#define KEEP_SHOOT_TIME2 100 //�ж������ǰҡʱ��(����)
#define ONECE_TRI_TIME 1000  //���̲�һ�εļ��ʱ��
#define KEEP_TRI_TIME1 250    //���̲�һ�εļ��ʱ��(����)(ң����)
#define KEEP_TRI_TIME2 100    //���̲�һ�εļ��ʱ��(����)(����)
#define ONECE_TRI_ANGLE 1296 //���̲�һ�εĽǶ�
#define FRI_STOP_ANGLE_REF 0
#define SHOOT_SPEED_STOP_OUT 0
#define _17mm_Heat_of_Projectile 10
typedef enum
{
	SHOOT_WEAK,
	SHOOT_ONECE,             //����
	SHOOT_KEEP,              //����
	SHOOT_AIM,               //����ģʽ
	TAKE_BULLET,             //��������
	SHOOT_BUFF,              //���
}SHOOT_MODE;

typedef enum //ģʽ��־λ
{
	KeyBord,
	Handle,
	
}USE_MODE;

typedef enum
{
  Tri_ON,
	Tri_OFF,
	
}SHOOT_TIME;

typedef enum
{
  KEEP_YES,
  KEEP_NO,
	
}KEEP_YES_NO;

typedef enum
{
  STOPPAGE,                //��ת
	DOWN_ON,                 //����
	NOMAL,                   //����
	
}PROTECT_MODE;

typedef struct
{
	int8_t buff_pitch_offset;
	int8_t buff_yaw_offset;
	
}GK_SHOOT;

typedef struct
{
 SHOOT_MODE Shoot_mode;
 SHOOT_MODE Last_Shoot_mode;
 GK_SHOOT BUFF_OFFSET;
 SHOOT_TIME Shoot_Time;
 KEEP_YES_NO KEEP_MODE;
 PROTECT_MODE Shoot_Protect;
 USE_MODE Use_Mode;
}All_Shoot_Mode;


typedef struct
{
 Speed_PidTypeDef_t Shoot_SPEED_PID;
 Angle_PidTypeDef_t Shoot_ANGLE_PID;
 Speed_PidTypeDef_t Fri_SPEED_PID1;
 Speed_PidTypeDef_t Fri_SPEED_PID2;
 Angle_PidTypeDef_t Fri_ANGLE_PID1;
 Angle_PidTypeDef_t Fri_ANGLE_PID2;
}SHOOT_PID;

typedef struct
{
char Chassis_BuDan_Flag;     //���̲������ٱ�־
char Shoot_More_Flag;        //������־
}Shoot_CAN2_Send;

typedef struct
{
 const RC_Ctrl_t *Get_Rc;
// const Referee_Data*CAN2_RC_DATA;
 const motor_measure_t *TRI_MOTOR;
 const motor_measure_t *FRI_MOTOR1;
 const motor_measure_t *FRI_MOTOR2;
 Shoot_CAN2_Send Shoot_Can2_Send;
 SHOOT_PID Shoot_PID;
 All_Shoot_Mode Shoot_Mode;
// PC_GET_DATA *gk_DATA;
// GIMBAL_Task *Get_Gimbal_Data;
}Shoot_DATA;

void Shoot_Init(Shoot_DATA *SHOOT_INIT);
void Shoot_Mode_Set(Shoot_DATA *SHOOT_MODE_SET);
void Shoot_Updata(Shoot_DATA *SHOOT_UPDATA);
void Shoot_move(Shoot_DATA *SHOOT_PID);
void Shoot_GiveCUR(Shoot_DATA *SHOOT_CUR);
void Shoot_Data_NIMING(Shoot_DATA *SHOOT_NIMING);
Shoot_DATA *Get_Shoot_DATA(void);
 float Shoot_Kaluli_limit(Shoot_DATA *SHOOT_heat_limt);
#endif

