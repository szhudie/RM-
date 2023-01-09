/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       pid.c/h
  * @brief      pid实现函数，包括初始化，PID计算函数，
  */

#include "pid.h"

pid_type_def Yz_speed_pid; 
pid_type_def Yz_angle_pid;
pid_type_def Pz_speed_pid;
pid_type_def Pz_angle_pid;
pid_type_def chassis_speed_pid[4];//定义底盘结构体
pid_type_def chassis_angle_pid[4];//定义底盘结构体
const motor_measure_t *motor_data;	//声明电机结构体指针(用于存储can读数)
const pid_type_def *pid_speed;
const pid_type_def *pid_angld;
extern int s0_flag;
int32_t xset;
//限制最大输出
//void LimitMax(pid_type_def *input, fp32 mode)   \
//    {    
//    if(mode==0){
//        if (input->Iout > input->max_iout)       
//        {                      
//            input->Iout = input->max_iout;       
//        }                      
//        else if (input->Iout < -(input->max_iout)) {
//					
//            input->Iout = -(input->max_iout);
//        }    
//			}				
//		if(mode==1){
//		        if (input->out > input->max_out)       
//        {                      
//            input->out = input->max_out;       
//        }                      
//        else if (input->out < -(input->max_out)) {
//					
//            input->out = -(input->max_out);
//        }    
//		}
//}
#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }
/**
  * @brief          pid struct data init
  * @param[out]     pid: PID结构数据指针
  * @param[in]      mode: PID_POSITION:普通PID
  *                 PID_DELTA: 差分PID
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid最大输出
  * @param[in]      max_iout: pid最大积分输出
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

void SPEED_PID_Init(Speed_PidTypeDef_t *pid, const fp32 PID[3], fp32 max_out, fp32 max_iout)
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}


/**
  * @brief          pid 输出清除
  * @param[out]     pid: PID结构数据指针
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

//速度环PID参数计算	
void Speed_Pid_Calc(Speed_PidTypeDef_t *pid)
{
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
		
    pid->error[0] = pid->ref - pid->fdb;
		
    pid->Pout = pid->Kp * pid->error[0];
    pid->Iout += pid->Ki * pid->error[0];
			
    pid->Dbuf[2] = pid->Dbuf[1];
    pid->Dbuf[1] = pid->Dbuf[0];
    pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
			
    pid->Dout = pid->Kd * pid->Dbuf[0];
    LimitMax(pid->Iout, pid->max_iout);
			
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    LimitMax(pid->out, pid->max_out);
		
}
/**
  * @brief          pid计算
  * @param[out]     pid: PID结构数据指针
  * @param[in]      ref: 反馈数据
  * @param[in]      set: 设定值
  * @retval         pid输出
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
    LimitMax(pid->Iout, pid->max_iout);
			
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    LimitMax(pid->out, pid->max_out);

    return pid->out;
}

//角度环PID初始化
void Angle_Pid_Init(Angle_PidTypeDef_t *pid,  const fp32 PID[3], fp32 max_out, fp32 max_iout)
{
    if (pid == NULL)
    {
       return;
    }
    pid->kp = PID[0];
    pid->ki = PID[1];
    pid->kd = PID[2];
		
    pid->ref = 0.0f;
    pid->fdb = 0.0f;

    pid->max_iout = max_iout;
    pid->max_out = max_out;
		    
		pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}
/*********************************************************************/

//角度环PID计算
void Angle_Pid_Cala(Angle_PidTypeDef_t *pid, fp32 fdb, fp32 ref)		//与速度环的微分计算不同
{
    if (pid == NULL)
    {
        return;
    }
		
		pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->fdb = fdb;
    pid->ref = ref;
    pid->error[0] = pid->ref - pid->fdb;
		pid->err = pid->ref - pid->fdb;
		
		pid->Pout = pid->kp * pid->err;
		
		if(__fabs(pid->err)>0.05f)
		{
			pid->Iout += pid->ki * pid->err;
		}
		pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
		pid->Dout = pid->kd * pid->Dbuf[0];
    LimitMax(pid->Iout, pid->max_iout);
			
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    LimitMax(pid->out, pid->max_out);
}

void angle_PID_clear(Angle_PidTypeDef_t *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->err=pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->ref =0.0f;
}




