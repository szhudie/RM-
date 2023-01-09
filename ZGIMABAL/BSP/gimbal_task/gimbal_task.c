#include "Gimbal_Task.h"
#include "pid.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "pid.h"
#include "Get_BMI088_Data.h"
#include "detect_tesk.h"
/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       gimbal_task.c/h
  * @brief    
  */
void rc_deadline_limit(int16_t input,int16_t *output,int16_t dealine);
void GIMBAL_Mode_Change_Control_Transit(Gimbal_Control_t *gimbal_mode_change);
void GIMBAL_Feedback_Update(Gimbal_Control_t *gimbal_feedback_update);
void gimbal_behaviour_control_set(fp32 *add_yaw, fp32 *add_pitch, Gimbal_Control_t *gimbal_control_set);
void GIMBAL_Set_Contorl(Gimbal_Control_t *gimbal_set_control);
void GIMBAL_Control_loop(Gimbal_Control_t *gimbal_control); 
void gimbal_motor_raw_angle_control(Gimbal_Motor_t *gimbal_motor);
void gimbal_motor_absolute_angle_control(Gimbal_Motor_t *gimbal_motor);
void gimbal_motor_relative_angle_control(Gimbal_Motor_t *gimbal_motor);
fp32 GIMBAL_PID_Calc(Gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta);
fp32 fp32_constrain(fp32 *Value, fp32 minValue, fp32 maxValue);
//��̨���������������
Gimbal_Control_t gimbal_control;
Gimbal_Control_t gimbal_control2;
//���͵�can ָ��
static int16_t Yaw_Can_Set_Current = 0, Pitch_Can_Set_Current = 0, Shoot_Can_Set_Current = 0;
fp32 Pitch_speed_pid[3] = {320,0,0};
fp32 Yaw_speed_pid[3] = {320,0,100};
fp32 YAW_GYRO_ABSOLUTE_PID_KP= 6.0f;
fp32 YAW_GYRO_ABSOLUTE_PID_KI =0.0f;
fp32 YAW_GYRO_ABSOLUTE_PID_KD =300.0f;
fp32 YAW_max_out = 66;
fp32 YAW_gyro_max_iout = 500;
fp32 YAW_gyro_max_out = 30000;

fp32 pitch_GYRO_ABSOLUTE_PID_KP =10.0f;
fp32 pitch_GYRO_ABSOLUTE_PID_KI =0.01f;
fp32 pitch_GYRO_ABSOLUTE_PID_KD =300.0f;
fp32 pitch_max_out = 100;
fp32 pitch_gyro_max_iout=2000;
fp32 pitch_gyro_out=30000;
fp32 pitch_max_iout = 38;
void gimbal_task(void const * argument)
{
	GIMBAL_Init(&gimbal_control);
	
  
  for(;;)
  {
		gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid.Kp=Yaw_speed_pid[0];
		gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid.Ki=Yaw_speed_pid[1];
		gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid.Kd=Yaw_speed_pid[2];
		gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid.max_out = YAW_gyro_max_out;
		gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid.max_iout = YAW_gyro_max_iout;
		gimbal_control.gimbal_yaw_motor.gimbal_motor_absolute_angle_pid.kp=YAW_GYRO_ABSOLUTE_PID_KP;
		gimbal_control.gimbal_yaw_motor.gimbal_motor_absolute_angle_pid.ki=YAW_GYRO_ABSOLUTE_PID_KI;
		gimbal_control.gimbal_yaw_motor.gimbal_motor_absolute_angle_pid.kd=YAW_GYRO_ABSOLUTE_PID_KD;
		gimbal_control.gimbal_yaw_motor.gimbal_motor_absolute_angle_pid.max_out=YAW_max_out;
		
		
		gimbal_control.gimbal_pitch_motor.gimbal_motor_gyro_pid.Kp=Pitch_speed_pid[0];
		gimbal_control.gimbal_pitch_motor.gimbal_motor_gyro_pid.Ki=Pitch_speed_pid[1];
		gimbal_control.gimbal_pitch_motor.gimbal_motor_gyro_pid.Kd=Pitch_speed_pid[2];
		gimbal_control.gimbal_pitch_motor.gimbal_motor_absolute_angle_pid.kp = pitch_GYRO_ABSOLUTE_PID_KP;
		gimbal_control.gimbal_pitch_motor.gimbal_motor_absolute_angle_pid.ki = pitch_GYRO_ABSOLUTE_PID_KI;
		gimbal_control.gimbal_pitch_motor.gimbal_motor_absolute_angle_pid.kd = pitch_GYRO_ABSOLUTE_PID_KD;
		gimbal_control.gimbal_pitch_motor.gimbal_motor_absolute_angle_pid.max_out = pitch_max_out;
		gimbal_control.gimbal_pitch_motor.gimbal_motor_gyro_pid.max_iout=pitch_gyro_max_iout;
		gimbal_control.gimbal_pitch_motor.gimbal_motor_gyro_pid.max_out=pitch_gyro_out;
		gimbal_control.gimbal_pitch_motor.gimbal_motor_absolute_angle_pid.max_iout = pitch_max_iout;
		
		
    gimbal_behaviour_mode_set(&gimbal_control);//ģʽ����
    GIMBAL_Mode_Change_Control_Transit(&gimbal_control);//���ݹ���
    GIMBAL_Feedback_Update(&gimbal_control);//��̨���ݷ���
    GIMBAL_Set_Contorl(&gimbal_control);                 //������̨������
    GIMBAL_Control_loop(&gimbal_control);                //��̨����PID����
    Yaw_Can_Set_Current = gimbal_control.gimbal_yaw_motor.given_current;
		Pitch_Can_Set_Current = gimbal_control.gimbal_pitch_motor.given_current;

		CAN_cmd_gimbal(Yaw_Can_Set_Current,Pitch_Can_Set_Current,0,0);
    osDelay(1);
  }
 
}

static gimbal_behaviour_e gimbal_behaviour = GIMBAL_ZERO_FORCE;
void ECD_Format(int *ecd){
        if ((*ecd) > 8191)  \
            (*ecd) -= 8191; \
        else if ((*ecd) < 0)     \
            (*ecd) += 8191;
}

//pid��������
static void Gimbal_PID_clear(Gimbal_PID_t *gimbal_pid_clear)
{
    if (gimbal_pid_clear == NULL)
    {
        return;
    }
    gimbal_pid_clear->err = gimbal_pid_clear->set = gimbal_pid_clear->get = 0.0f;
    gimbal_pid_clear->out = gimbal_pid_clear->Pout = gimbal_pid_clear->Iout = gimbal_pid_clear->Dout = 0.0f;
}


//��ʼ��pid ����ָ��
static void GIMBAL_Init(Gimbal_Control_t *gimbal_init)
{
    //�������ָ���ȡ
    gimbal_init->gimbal_yaw_motor.gimbal_motor_measure = get_Yaw_Gimbal_Motor_Measure_Point();
    gimbal_init->gimbal_pitch_motor.gimbal_motor_measure = get_Pitch_Gimbal_Motor_Measure_Point();
    //����������ָ���ȡ
    gimbal_init->gimbal_INT_gyro_point = GET_GYRO_DATA();
    //ң��������ָ���ȡ
    gimbal_init->gimbal_rc_ctrl = get_remote_control_point();
    //��ʼ�����ģʽ
    gimbal_init->gimbal_yaw_motor.gimbal_motor_mode = gimbal_init->gimbal_yaw_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    gimbal_init->gimbal_pitch_motor.gimbal_motor_mode = gimbal_init->gimbal_pitch_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    //��ʼ��yaw���pid
    GIMBAL_PID_Init(&gimbal_init->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid, YAW_GYRO_ABSOLUTE_PID_MAX_OUT, YAW_GYRO_ABSOLUTE_PID_MAX_IOUT, YAW_GYRO_ABSOLUTE_PID_KP, YAW_GYRO_ABSOLUTE_PID_KI, YAW_GYRO_ABSOLUTE_PID_KD);
    GIMBAL_PID_Init(&gimbal_init->gimbal_yaw_motor.gimbal_motor_relative_angle_pid, YAW_ENCODE_RELATIVE_PID_MAX_OUT, YAW_ENCODE_RELATIVE_PID_MAX_IOUT, YAW_ENCODE_RELATIVE_PID_KP, YAW_ENCODE_RELATIVE_PID_KI, YAW_ENCODE_RELATIVE_PID_KD);
    PID_init(&gimbal_init->gimbal_yaw_motor.gimbal_motor_gyro_pid, 1, Yaw_speed_pid, YAW_SPEED_PID_MAX_OUT, YAW_SPEED_PID_MAX_IOUT);
    //��ʼ��pitch���pid
    GIMBAL_PID_Init(&gimbal_init->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid, PITCH_GYRO_ABSOLUTE_PID_MAX_OUT, PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT, PITCH_GYRO_ABSOLUTE_PID_KP, PITCH_GYRO_ABSOLUTE_PID_KI, PITCH_GYRO_ABSOLUTE_PID_KD);
    GIMBAL_PID_Init(&gimbal_init->gimbal_pitch_motor.gimbal_motor_relative_angle_pid, PITCH_ENCODE_RELATIVE_PID_MAX_OUT, PITCH_ENCODE_RELATIVE_PID_MAX_IOUT, PITCH_ENCODE_RELATIVE_PID_KP, PITCH_ENCODE_RELATIVE_PID_KI, PITCH_ENCODE_RELATIVE_PID_KD);
    PID_init(&gimbal_init->gimbal_pitch_motor.gimbal_motor_gyro_pid, 1, Pitch_speed_pid, PITCH_SPEED_PID_MAX_OUT, PITCH_SPEED_PID_MAX_IOUT);
    GIMBAL_Feedback_Update(&gimbal_control);//��̨���ݷ���
		//�趨ֵ���ڵ�ǰֵ ��ֹ��ʼ���ת��
    gimbal_init->gimbal_yaw_motor.absolute_angle_set = gimbal_init->gimbal_yaw_motor.absolute_angle;
    gimbal_init->gimbal_yaw_motor.relative_angle_set = gimbal_init->gimbal_yaw_motor.relative_angle;
    gimbal_init->gimbal_yaw_motor.motor_gyro_set = gimbal_init->gimbal_yaw_motor.motor_gyro;

    gimbal_init->gimbal_pitch_motor.absolute_angle_set = gimbal_init->gimbal_pitch_motor.absolute_angle;
    gimbal_init->gimbal_pitch_motor.relative_angle_set = gimbal_init->gimbal_pitch_motor.relative_angle;
    gimbal_init->gimbal_pitch_motor.motor_gyro_set = gimbal_init->gimbal_pitch_motor.motor_gyro;
}

static void GIMBAL_PID_Init(Gimbal_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd)
{
    if (pid == NULL)
    {
        return;
    }
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->err = 0.0f;
    pid->get = 0.0f;
    pid->set = 0.0f;
		
    pid->max_iout = max_iout;
    pid->max_out = maxout;
		pid->Pout = 0.0f;
		pid->Iout = 0.0f;
		pid->Dout = 0.0f;
		pid->out = 0.0f;
}

//����ģʽ��ͬѡ��ͬ�������ʽ(ֱ������������ǡ����)
void gimbal_behaviour_mode_set(Gimbal_Control_t *gimbal_mode_set)
{
    if (gimbal_mode_set == NULL)
    {
        return;
    }
    //����ң����ѡ��ģʽ
    gimbal_behavour_set(gimbal_mode_set);
		//����ģʽѡ�����ģʽ
    if (gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    }
    else if (gimbal_behaviour == GIMBAL_INIT)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    }
    else if (gimbal_behaviour == GIMBAL_CALI)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    }
    else if (gimbal_behaviour == GIMBAL_ABSOLUTE_ANGLE)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
    }
    else if (gimbal_behaviour == GIMBAL_RELATIVE_ANGLE)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    }
    else if (gimbal_behaviour == GIMBAL_MOTIONLESS)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    }
		
}


static void gimbal_behavour_set(Gimbal_Control_t *gimbal_mode_set)
{

    //���ؿ��� ��̨״̬
    if (switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[ModeChannel]))
    {
        gimbal_behaviour = GIMBAL_ZERO_FORCE;//����
    }
    else if (switch_is_mid(gimbal_mode_set->gimbal_rc_ctrl->rc.s[ModeChannel]))
    {
        gimbal_behaviour = GIMBAL_RELATIVE_ANGLE;//���
    }
    else if (switch_is_up(gimbal_mode_set->gimbal_rc_ctrl->rc.s[ModeChannel]))
    {
        gimbal_behaviour = GIMBAL_ABSOLUTE_ANGLE;//����
    }

		if(Detect_Judeg(2)||Detect_Judeg(1)||Detect_Judeg(0))
    {
			gimbal_behaviour = GIMBAL_ZERO_FORCE;//����
		}
}

//ÿ�α任ģʽ�������趨ֵ���ڵ�ǰֵ
static void GIMBAL_Mode_Change_Control_Transit(Gimbal_Control_t *gimbal_mode_change)
{
    if (gimbal_mode_change == NULL)
    {
        return;
    }
    //yaw���״̬���л���������
    if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_RAW && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_mode_change->gimbal_yaw_motor.raw_cmd_current = gimbal_mode_change->gimbal_yaw_motor.current_set = gimbal_mode_change->gimbal_yaw_motor.given_current;
    }
    else if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_GYRO && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_mode_change->gimbal_yaw_motor.absolute_angle_set = gimbal_mode_change->gimbal_yaw_motor.absolute_angle;
    }
    else if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_ENCONDE && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_mode_change->gimbal_yaw_motor.absolute_angle_set = gimbal_mode_change->gimbal_yaw_motor.absolute_angle;
    }
    gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode = gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode;

    //pitch���״̬���л���������
    if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_RAW && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_mode_change->gimbal_pitch_motor.raw_cmd_current = gimbal_mode_change->gimbal_pitch_motor.current_set = gimbal_mode_change->gimbal_pitch_motor.given_current;
    }
    else if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_GYRO && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_mode_change->gimbal_pitch_motor.absolute_angle_set = gimbal_mode_change->gimbal_pitch_motor.absolute_angle;
    }
    else if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_ENCONDE && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_mode_change->gimbal_pitch_motor.absolute_angle_set = gimbal_mode_change->gimbal_pitch_motor.absolute_angle;
    }

    gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode = gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode;
}

// �������ǵ�ֵ������̨�ṹ��
static void GIMBAL_Feedback_Update(Gimbal_Control_t *gimbal_feedback_update)
{
    if (gimbal_feedback_update == NULL)
    {
       return;
    }
  gimbal_feedback_update->gimbal_yaw_motor.absolute_angle = gimbal_feedback_update->gimbal_INT_gyro_point->GET_YAW;
  gimbal_feedback_update->gimbal_pitch_motor.absolute_angle= gimbal_feedback_update->gimbal_INT_gyro_point->GET_ROW;
  gimbal_feedback_update->gimbal_pitch_motor.motor_gyro=gimbal_feedback_update->gimbal_pitch_motor.gimbal_motor_measure->speed_rpm;
	gimbal_feedback_update->gimbal_yaw_motor.motor_gyro=gimbal_feedback_update->gimbal_yaw_motor.gimbal_motor_measure->speed_rpm;
	gimbal_feedback_update->gimbal_pitch_motor.relative_angle=gimbal_feedback_update->gimbal_pitch_motor.gimbal_motor_measure->ecd;
	gimbal_feedback_update->gimbal_yaw_motor.relative_angle=gimbal_feedback_update->gimbal_yaw_motor.gimbal_motor_measure->ecd_out;
}

//P���޷�����
fp32 fp32_constrain(fp32 *Value, fp32 minValue, fp32 maxValue)
{
    if (*Value < minValue){
			*Value=minValue;
        return *Value;}
    else if (*Value > maxValue){
			*Value=maxValue;
        return *Value;}
    else
        return *Value;
}

//��̨����������
static void GIMBAL_Set_Contorl(Gimbal_Control_t *gimbal_set_control)
{
    if (gimbal_set_control == NULL)
    {
        return;
    }

    fp32 add_yaw_angle = 0.0f;
    fp32 add_pitch_angle = 0.0f;
    gimbal_behaviour_control_set(&add_yaw_angle, &add_pitch_angle, gimbal_set_control);
    //yaw���ģʽ����
    if (gimbal_set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        //rawģʽ�£�ֱ�ӷ��Ϳ���ֵ
        gimbal_set_control->gimbal_yaw_motor.raw_cmd_current = add_yaw_angle;
    }
    else if (gimbal_set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {   
				//encondeģʽ�£��������Ƕȿ���
				gimbal_set_control->gimbal_yaw_motor.absolute_angle_set+=add_yaw_angle/2000.0f;
    }
    else if (gimbal_set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        //gyroģʽ�£������ǽǶȿ���
				gimbal_set_control->gimbal_yaw_motor.absolute_angle_set+=add_yaw_angle/2000.0f;
    }
		
    //pitch���ģʽ����
		if (gimbal_set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        //rawģʽ�£�ֱ�ӷ��Ϳ���ֵ
        gimbal_set_control->gimbal_pitch_motor.raw_cmd_current = add_pitch_angle;
    }
		else if (gimbal_set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
			gimbal_set_control->gimbal_pitch_motor.absolute_angle_set +=add_pitch_angle/5000.0f;
        //gyroģʽ�£������ǽǶȿ���
       gimbal_set_control->gimbal_pitch_motor.absolute_angle_set=fp32_constrain(&gimbal_set_control->gimbal_pitch_motor.absolute_angle_set,-38,20);
    }
    else if (gimbal_set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
			gimbal_set_control->gimbal_pitch_motor.absolute_angle_set +=add_pitch_angle/5000.0f;
        //gyroģʽ�£������ǽǶȿ���
       gimbal_set_control->gimbal_pitch_motor.absolute_angle_set=fp32_constrain(&gimbal_set_control->gimbal_pitch_motor.absolute_angle_set,-38,20);
    }
}
/**
  * @brief          ң�����������жϣ���Ϊң�����Ĳ�������λ��ʱ�򣬲�һ���Ƿ���1024������
  * @author         RM
  * @param[in]      �����ң����ֵ
  * @param[in]      ��������������ң����ֵ
  * @param[in]      ����ֵ
  * @retval         ���ؿ�
  */
void rc_deadline_limit(int16_t input,int16_t *output,int16_t dealine){
  if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (*output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (*output) = 0;                                \
        }           

}

/**
  * @brief          ��̨��Ϊ���ƣ����ݲ�ͬ��Ϊ���ò�ͬ���ƺ���
  * @author         RM
  * @param[in]      ���õ�yaw�Ƕ�����ֵ����λ rad
  * @param[in]      ���õ�pitch�Ƕ�����ֵ����λ rad
  * @param[in]      ��̨����ָ��
  * @retval         ���ؿ�
  */
void gimbal_behaviour_control_set(fp32 *add_yaw, fp32 *add_pitch, Gimbal_Control_t *gimbal_control_set)
{

    if (add_yaw == NULL || add_pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    static fp32 rc_add_yaw, rc_add_pit;
    static int16_t yaw_channel = 0, pitch_channel = 0;
    //��ң���������ݴ������� int16_t yaw_channel,pitch_channel
    rc_deadline_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[YawChannel], &yaw_channel, 5);
    rc_deadline_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[PitchChannel], &pitch_channel, 5);


    if (gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        rc_add_yaw=0;
			  rc_add_pit=0;
    }
    else if (gimbal_behaviour == GIMBAL_INIT)
    {
       
    }
    else if (gimbal_behaviour == GIMBAL_ABSOLUTE_ANGLE)
    {
    rc_add_yaw = -yaw_channel ;
    rc_add_pit = -pitch_channel ; 
    }
    else if (gimbal_behaviour == GIMBAL_RELATIVE_ANGLE)
    {
    rc_add_yaw = -yaw_channel ;
    rc_add_pit = -pitch_channel ;
    }
    //��������������ֵ
    *add_yaw = rc_add_yaw;
    *add_pitch = rc_add_pit;
}

//��̨����״̬ʹ�ò�ͬ����pid
static void GIMBAL_Control_loop(Gimbal_Control_t *gimbal_control_loop)
{
    if (gimbal_control_loop == NULL)
    {
        return;
    }
    //yaw��ͬģʽ���ڲ�ͬ�Ŀ��ƺ���
    if (gimbal_control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        //raw����
        gimbal_motor_raw_angle_control(&gimbal_control_loop->gimbal_yaw_motor);
    }
    else if (gimbal_control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        //gyro�Ƕȿ���
        gimbal_motor_absolute_angle_control(&gimbal_control_loop->gimbal_yaw_motor);
    }
    else if (gimbal_control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        //enconde�Ƕȿ���
        gimbal_motor_relative_angle_control(&gimbal_control_loop->gimbal_yaw_motor);
    }

    //pitch��ͬģʽ���ڲ�ͬ�Ŀ��ƺ���
    if (gimbal_control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        //raw����
        gimbal_motor_raw_angle_control(&gimbal_control_loop->gimbal_pitch_motor);
    }
    else if (gimbal_control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        //gyro�Ƕȿ���
        gimbal_motor_absolute_angle_control(&gimbal_control_loop->gimbal_pitch_motor);
    }
    else if (gimbal_control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        //enconde�Ƕȿ���
        gimbal_motor_relative_angle_control(&gimbal_control_loop->gimbal_pitch_motor);
    }
}
// �����Ƿ��� ����PID
static void gimbal_motor_absolute_angle_control(Gimbal_Motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    //�ǶȻ����ٶȻ�����pid����
    gimbal_motor->motor_gyro_set = GIMBAL_PID_Calc(&gimbal_motor->gimbal_motor_absolute_angle_pid, gimbal_motor->absolute_angle, gimbal_motor->absolute_angle_set, gimbal_motor->motor_gyro);
    gimbal_motor->current_set = PID_calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);
    //����ֵ��ֵ
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}
// ������� ����PID
static void gimbal_motor_relative_angle_control(Gimbal_Motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    //�ǶȻ����ٶȻ�����pid����
    gimbal_motor->motor_gyro_set = GIMBAL_PID_Calc(&gimbal_motor->gimbal_motor_absolute_angle_pid, gimbal_motor->absolute_angle, gimbal_motor->absolute_angle_set, gimbal_motor->motor_gyro);
    gimbal_motor->current_set = PID_calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);
    //����ֵ��ֵ
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}

//ֱ����� ����PID
static void gimbal_motor_raw_angle_control(Gimbal_Motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    gimbal_motor->current_set = gimbal_motor->raw_cmd_current;
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}

//����������
void minMax(Gimbal_PID_t *input, fp32 mode)   \
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

static fp32 GIMBAL_PID_Calc(Gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->get = get;
    pid->set = set;

        pid->error[0] = set - get;
        pid->Pout = pid->kp * pid->error[0];
        pid->Iout += pid->ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->kd * pid->Dbuf[0];
        minMax(pid, 0);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        minMax(pid, 1);

    return pid->out;
}
