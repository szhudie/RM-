/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       chassis.c/h
  * @brief      ��ɵ��̿�������
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
#include "chassis_task.h"
#include "detect_tesk.h"


#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "pid.h"
#include "chassis_behaviour.h"
#include "bsp_can.h"
#include "arm_math.h"
#include "user_lib.h"

#define rc_deadline_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }

//�����˶�����
chassis_move_t chassis_move;
//���̳�ʼ������Ҫ��pid��ʼ��
static void chassis_init(chassis_move_t *chassis_move_init);
//����״̬��ѡ��ͨ��ң�����Ŀ���
static void chassis_set_mode(chassis_move_t *chassis_move_mode);
//�������ݸ���
static void chassis_feedback_update(chassis_move_t *chassis_move_update);
//����״̬�ı����������ĸı�static
void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit);
//�������ø���ң����������
static void chassis_set_contorl(chassis_move_t *chassis_move_control);
//����PID�����Լ��˶��ֽ�
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop);

 fp32 motor_speed_pid[3] = {700,0.3,0};
 fp32 motor_angle_pid[3] = {1.9,0,0};
 fp32 M3505_MOTOR_SPEED_PID_MAX_OUT=10000.0f;
fp32 M3505_MOTOR_SPEED_PID_MAX_IOUT=500.0f;
 fp32 CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT=100.0f;
fp32 CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT=0.2f;
 fp32 CHASSIS_WZ_SET_SCALE = 0.0f;
 fp32 CHASSIS_WZ_SET_YOU = 0.0f;
//������
void chassis_task(void *pvParameters)
{
    //����һ��ʱ��
    vTaskDelay(CHASSIS_TASK_INIT_TIME);
    //���̳�ʼ��
    chassis_init(&chassis_move);

    //�жϵ��̵���Ƿ�����
    while (1)
    {
        //ң��������״̬
        chassis_set_mode(&chassis_move);
        //ң����״̬�л����ݱ���
        chassis_mode_change_control_transit(&chassis_move);
        //�������ݸ���
        chassis_feedback_update(&chassis_move);
        //���̿���������
        chassis_set_contorl(&chassis_move);
        //���̿���PID����
        chassis_control_loop(&chassis_move);
				//���ջ��������Ƕ���,YAW����ߣ���̨����
		if(Detect_Judeg(5)||Detect_Judeg(0)||Detect_Judeg(1)||Detect_Judeg(2)||Detect_Judeg(3)||Detect_Judeg(4))
		{
			CAN_cmd_chassis(0,0,0,0);
		}
		else
		{
			CAN_cmd_chassis(chassis_move.motor_chassis[0].give_current, chassis_move.motor_chassis[1].give_current,
                                chassis_move.motor_chassis[2].give_current, chassis_move.motor_chassis[3].give_current);
		}
        //ϵͳ��ʱ
        vTaskDelay(CHASSIS_CONTROL_TIME_MS);

    }
}

static void chassis_init(chassis_move_t *chassis_move_init)
{
    if (chassis_move_init == NULL)
    {
        return;
    }
    uint8_t i;

    //���̿���״̬Ϊֹͣ
    chassis_move_init->chassis_mode = CHASSIS_VECTOR_RAW;//����
    //��ȡң����ָ��
    chassis_move_init->chassis_RC = get_remote_control_point();
    //��ȡ��������̬��ָ��
    //��ȡ��̨�������ָ��
    chassis_move_init->chassis_yaw_motor = get_Yaw_Gimbal_Motor_Measure_Point();
    //��ʼ��PID �˶�
    for (i = 0; i < 4; i++)
    {
        chassis_move_init->motor_chassis[i].chassis_motor_measure = get_Chassis_Motor_Measure_Point(i);
        PID_init(&chassis_move_init->motor_speed_pid[i], PID_POSITION, motor_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
    }
    //��ʼ����תPID
    PID_init(&chassis_move_init->chassis_angle_pid, PID_POSITION, motor_angle_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);
    //��һ���˲�����б����������
    //��� ��С�ٶ�
    chassis_move_init->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
    chassis_move_init->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;

    chassis_move_init->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
    chassis_move_init->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;

    //����һ������
    chassis_feedback_update(chassis_move_init);
}

static void chassis_set_mode(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }

    chassis_behaviour_mode_set(chassis_move_mode);
}

static void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit)
{
    if (chassis_move_transit == NULL)
    {
        return;
    }

    if (chassis_move_transit->last_chassis_mode == chassis_move_transit->chassis_mode)
    {
        return;
    }

    //���������̨ģʽ
    if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {
        chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
    }
    //���벻������̨ģʽ
    else if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_NO_FOLLOW_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
    {
        chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
    }

    chassis_move_transit->last_chassis_mode = chassis_move_transit->chassis_mode;
}

static void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
    if (chassis_move_update == NULL)
    {
        return;
    }
    static	int32_t total_ecd;
    uint8_t i = 0;
    for (i = 0; i < 4; i++)
    {
        //���µ���ٶȣ����ٶ����ٶȵ�PID΢��
			chassis_move_update->motor_chassis[i].speed = (chassis_move_update->motor_chassis[i].chassis_motor_measure->speed_rpm*PI/30.0f)/19.0f;//���ٶ�
    }

    //���µ���ǰ���ٶ� x�� ƽ���ٶ�y����ת�ٶ�wz������ϵΪ����ϵ

    //���������̬�Ƕ�, �����������������������ⲿ�ִ���
		total_ecd = chassis_move_update->chassis_yaw_motor->ecd_out - 3500;
		chassis_move_update->chassis_yaw = theta_format(total_ecd / (8192.0f/360.0f));
}

//ң���������ݴ���ɵ��̵�ǰ��vx�ٶȣ�vy�ٶ�
void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL)
    {
        return;
    }
    //ң����ԭʼͨ��ֵ
    int16_t vx_channel, vy_channel;
    fp32 vx_set_channel, vy_set_channel;
    //�������ƣ���Ϊң�������ܴ��ڲ��� ҡ�����м䣬��ֵ��Ϊ0
    rc_deadline_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
    rc_deadline_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE);

			vx_set_channel = (vx_channel * CHASSIS_VX_RC_SEN);
			vy_set_channel = (vy_channel * CHASSIS_VY_RC_SEN);

    //ֹͣ�źţ�����Ҫ�������٣�ֱ�Ӽ��ٵ���
		if(vx_channel==0&&vy_channel==0)
		{
			*vx_set=*vy_set=0;
		}
		
		*vx_set = vx_set_channel;
    *vy_set = vy_set_channel;

}

//����ң�������������
static void chassis_set_contorl(chassis_move_t *chassis_move_control)
{
    
    if (chassis_move_control == NULL)
    {
        return;
    }
    static float d_theta = 0;
    //�����ٶ�
    fp32 vx_set = 0.0f, vy_set = 0.0f, angle_set = 0.0f;
    chassis_behaviour_control_set(&vx_set, &vy_set, &angle_set, chassis_move_control);

    //������̨ģʽ
    if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {
        fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
        //��ת���Ƶ����ٶȷ��򣬱�֤ǰ����������̨�����������˶�ƽ��
			//��ת���Ƶ����ٶȷ��򣬱�֤ǰ����������̨�����������˶�ƽ��
			if(chassis_move_control->chassis_yaw < 0){
				d_theta =  chassis_move_control->chassis_yaw - 0;
			}
			else if(chassis_move_control->chassis_yaw <= 0+ 180 ){
				d_theta =chassis_move_control->chassis_yaw - 0;
			}
			else if(chassis_move_control->chassis_yaw <= 360){
				d_theta = 360 -  chassis_move_control->chassis_yaw + 0;
			}
			d_theta /= -57.295;//�Ƕ�ת����
			
			sin_yaw = arm_sin_f32(d_theta);
			cos_yaw = arm_cos_f32(d_theta);

			chassis_move_control->vx_set = cos_yaw * vx_set - sin_yaw * vy_set;
			chassis_move_control->vy_set = sin_yaw * vx_set + cos_yaw * vy_set;
        //���ÿ��������̨�Ƕ�
        chassis_move_control->chassis_relative_angle_set = angle_set;
        //������תPID���ٶ�
   			if(chassis_move_control->swing_flag==1)
			{
					static int swing_angle=45;
					static int sin_angle=25;
//					if(inclined_swing_flag==1)
//					{
//						chassis_move_control->wz_set = PID_Calc(&chassis_move_control->chassis_angle_pid,\
//						chassis_move_control->chassis_yaw_motor->relative_angle,swing_angle+Get_Sin(sin_angle,160));
//						if(time_add%1500==0)
//						{
//							swing_angle=-swing_angle;
//						}
//					}
//					else
//					{
						chassis_move_control->wz_set = (int)angle_set;
//					}
			}
			else
			{
					 chassis_move_control->wz_set = -PID_calc(&chassis_move_control->chassis_angle_pid,chassis_move_control->chassis_yaw, chassis_move_control->chassis_relative_angle_set); 
//				chassis_move_control->wz_set = 0;
			}
        //�ٶ��޷�
        chassis_move_control->vx_set = fp32_constrain(chassis_move_control->vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(chassis_move_control->vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
    }
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
    {

        //����������̨
        //���ģʽ�£��Ƕ����õ�Ϊ ���ٶ�
        fp32 chassis_wz = angle_set;
        chassis_move_control->wz_set = chassis_wz;
        chassis_move_control->vx_set = vx_set;
        chassis_move_control->vy_set = vy_set;
    }
}



static void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4])
{
    //��ת��ʱ�� ������̨��ǰ��������ǰ������ 0 ��1 ��ת���ٶȱ����� �������� 2,3 ��ת���ٶȱ��
    wheel_speed[0] = -vx_set - vy_set + (-CHASSIS_WZ_SET_SCALE - CHASSIS_WZ_SET_YOU - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[1] = vx_set - vy_set + (-CHASSIS_WZ_SET_SCALE + CHASSIS_WZ_SET_YOU - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[2] = vx_set + vy_set + (CHASSIS_WZ_SET_SCALE + CHASSIS_WZ_SET_YOU - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[3] = -vx_set + vy_set + (CHASSIS_WZ_SET_SCALE - CHASSIS_WZ_SET_YOU - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
}

static void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
    fp32 max_vector = 0.0f, vector_rate = 0.0f;
    fp32 temp = 0.0f;
    fp32 wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    uint8_t i = 0;
    //�����˶��ֽ�
    chassis_vector_to_mecanum_wheel_speed(chassis_move_control_loop->vx_set,
                                          chassis_move_control_loop->vy_set, chassis_move_control_loop->wz_set, wheel_speed);

if(chassis_move_control_loop->chassis_mode == CHASSIS_VECTOR_RAW){
for (i = 0; i < 4; i++){
chassis_move_control_loop->motor_chassis[i].give_current = 0;
}
}
else if(chassis_move_control_loop->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW){
    //����pid

    for (i = 0; i < 4; i++)
    {
			  chassis_move_control_loop->motor_chassis[i].speed_set = wheel_speed[i];
        PID_calc(&chassis_move_control_loop->motor_speed_pid[i], chassis_move_control_loop->motor_chassis[i].speed, chassis_move_control_loop->motor_chassis[i].speed_set);
    }

    //��ֵ����ֵ
    for (i = 0; i < 4; i++)
    {
        chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(chassis_move_control_loop->motor_speed_pid[i].out);
    }
	}
else if(chassis_move_control_loop->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW){

		
    for (i = 0; i < 4; i++)
    {
			  chassis_move_control_loop->motor_chassis[i].speed_set = wheel_speed[i];
        PID_calc(&chassis_move_control_loop->motor_speed_pid[i], chassis_move_control_loop->motor_chassis[i].speed, chassis_move_control_loop->motor_chassis[i].speed_set);
    }
			    for (i = 0; i < 4; i++)
    {
        chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(chassis_move_control_loop->motor_speed_pid[i].out);
    }
		
	}
}
