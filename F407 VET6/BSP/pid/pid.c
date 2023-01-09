/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       pid.c/h
  * @brief      pidʵ�ֺ�����������ʼ����PID���㺯����
  */

#include "pid.h"


pid_type_def chassis_speed_pid[4];//������̽ṹ��
pid_type_def chassis_angle_pid[4];//������̽ṹ��
const motor_measure_t *motor_data;	//��������ṹ��ָ��(���ڴ洢can����)
const pid_type_def *pid_speed;
const pid_type_def *pid_angld;
extern int s0_flag;
int32_t xset;


void LimitMax(pid_type_def *input, fp32 mode)   \
    {    
    if(mode==0){
        if (input->Iout > input->max_iout)       
        {                      
            input->Iout = input->max_iout;       
        }                      
        else if (input->Iout < -(input->max_iout)) {
					
            input->Iout = -(input->max_iout);
        }    
			}				
		if(mode==1){
		        if (input->out > input->max_out)       
        {                      
            input->out = input->max_out;       
        }                      
        else if (input->out < -(input->max_out)) {
					
            input->out = -(input->max_out);
        }    
		}
}

/**
  * @brief          pid struct data init
  * @param[out]     pid: PID struct data point
  * @param[in]      mode: PID_POSITION: normal pid
  *                 PID_DELTA: delta pid
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid max out
  * @param[in]      max_iout: pid max iout
  * @retval         none
  */
/**
  * @brief          pid struct data init
  * @param[out]     pid: PID�ṹ����ָ��
  * @param[in]      mode: PID_POSITION:��ͨPID
  *                 PID_DELTA: ���PID
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid������
  * @param[in]      max_iout: pid���������
  * @retval         none
  */
void PID_init(pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout)
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }
    pid->mode = mode;
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}

/**
  * @brief          pid calculate 
  * @param[out]     pid: PID struct data point
  * @param[in]      ref: feedback data 
  * @param[in]      set: set point
  * @retval         pid out
  */
/**
  * @brief          pid����
  * @param[out]     pid: PID�ṹ����ָ��
  * @param[in]      ref: ��������
  * @param[in]      set: �趨ֵ
  * @retval         pid���
  */

fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;

        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        LimitMax(pid, 0);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid, 1);

    return pid->out;
}


//fp32 YPID_calc(pid_type_def *pid, fp32 ref, RC_Ctrl_t *RC_CtrlData,fp32 set,int mode)
//{
//    if (pid == NULL)
//    {
//        return 0.0f;
//    }
//		
//		if(mode==1){
//   xset = RC_CtrlData->sex.yes;

//		}else if(mode==2){
//		
//		xset = RC_CtrlData->rc.ch[0];

//		}else if(mode == 0){
//			
//		xset = set;
//		
//		}
//    pid->error[2] = pid->error[1];
//    pid->error[1] = pid->error[0];
//    pid->set = xset;
//    pid->fdb = ref;
//    pid->error[0] = xset - ref;

//        pid->Pout = pid->Kp * pid->error[0];
//        pid->Iout += pid->Ki * pid->error[0];
//        pid->Dbuf[2] = pid->Dbuf[1];
//        pid->Dbuf[1] = pid->Dbuf[0];
//        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
//        pid->Dout = pid->Kd * pid->Dbuf[0];
//        LimitMax(pid, 0);
//        pid->out = pid->Pout + pid->Iout + pid->Dout;
//        LimitMax(pid, 1);

//    return pid->out;
//}


/**
  * @brief          pid out clear
  * @param[out]     pid: PID struct data point
  * @retval         none
  */
/**
  * @brief          pid ������
  * @param[out]     pid: PID�ṹ����ָ��
  * @retval         none
  */
void PID_clear(pid_type_def *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->set = 0.0f;
}






// ���Ƶ������
//void StartTask02(void *argument){
////if(s0_flag==1){
////	for (i = 0; i < 4; i++)//�ĸ����ѭ������PID
////{
////		YPID_calc(&chassis_angle_pid[i], motor_chassis[i].ecd_out, &RC_CtrlData,1);//----pid����
////}

////			for (i = 0; i < 4; i++)//�ĸ����ѭ������PID
////{
////		PID_calc(&chassis_speed_pid[i], motor_chassis[i].speed_rpm,chassis_angle_pid[i].out);//----pid����
////}
//////		CAN_cmd_chassis(chassis_speed_pid[0].out, //���÷��ͱ��ĺ�������pid���������ֵ���͸����
//////		                chassis_speed_pid[1].out, 
//////		                chassis_speed_pid[2].out, 
//////                    chassis_speed_pid[3].out);
////}
////else if(s0_flag==3){

////YPID_calc(&chassis_speed_pid[i], motor_chassis[i].speed_rpm,&RC_CtrlData,0);//----pid����

////}
//}



