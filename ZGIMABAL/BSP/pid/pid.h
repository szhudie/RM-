/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       pid.c/h
  * @brief      pidʵ�ֺ�����������ʼ����PID���㺯����
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef PID_H
#define PID_H
#include "struct_typedef.h"
#include "bsp_dma.h"
#include "bsp_can.h"

typedef  struct
{
    fp32 ref;      //�趨ֵ	
    fp32 fdb;			 //��ǰֵ

    fp32 Kp;       //PID ������
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;  //������
    fp32 max_iout; //���������

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
	
    fp32 Dbuf[3];  //΢���� 0���� 1��һ�� 2���ϴ�
    fp32 error[3]; //����� 0���� 1��һ�� 2���ϴ�

} Speed_PidTypeDef_t;
typedef struct
{
    uint8_t mode;
    //PID ������
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;  //������
    fp32 max_iout; //���������

    fp32 set;
    fp32 fdb;

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];  //΢���� 0���� 1��һ�� 2���ϴ�
    fp32 error[3]; //����� 0���� 1��һ�� 2���ϴ�

} pid_type_def;
typedef  struct
{  
    fp32 ref;  //�趨ֵ
    fp32 fdb;
    fp32 err;
	
    fp32 kp;
    fp32 ki;
    fp32 kd;

    fp32 max_out;
    fp32 max_iout;
	  fp32 max_dout;
	
    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
	
		fp32 Dbuf[3];  //΢���� 0���� 1��һ�� 2���ϴ�
    fp32 error[3]; //����� 0���� 1��һ�� 2���ϴ�

	  fp32 offset_Tri_ecd;
} Angle_PidTypeDef_t;
extern void PID_init(pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout);
extern fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set);
extern void PID_clear(pid_type_def *pid);
void math(RC_Ctrl_t *RC_CtrlData,int mode);
void Speed_Pid_Calc(Speed_PidTypeDef_t *pid);
void SPEED_PID_Init(Speed_PidTypeDef_t *pid, const fp32 PID[3], fp32 max_out, fp32 max_iout);
void Angle_Pid_Init(Angle_PidTypeDef_t *pid, const fp32 PID[3], fp32 max_out, fp32 max_iout);
void Angle_Pid_Cala(Angle_PidTypeDef_t *pid,fp32 fdb , fp32 ref);
extern void angle_PID_clear(Angle_PidTypeDef_t *pid);
extern pid_type_def chassis_speed_pid[4];//������̽ṹ��
extern pid_type_def chassis_angle_pid[4];//������̽ṹ��
extern pid_type_def Yz_speed_pid; 
extern pid_type_def Yz_angle_pid;
extern pid_type_def Pz_speed_pid;
extern pid_type_def Pz_angle_pid;

#endif