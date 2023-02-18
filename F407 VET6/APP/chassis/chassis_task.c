/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       chassis.c/h
  * @brief      完成底盘控制任务。
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
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

//底盘运动数据
chassis_move_t chassis_move;
//底盘初始化，主要是pid初始化
static void chassis_init(chassis_move_t *chassis_move_init);
//底盘状态机选择，通过遥控器的开关
static void chassis_set_mode(chassis_move_t *chassis_move_mode);
//底盘数据更新
static void chassis_feedback_update(chassis_move_t *chassis_move_update);
//底盘状态改变后处理控制量的改变static
void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit);
//底盘设置根据遥控器控制量
static void chassis_set_contorl(chassis_move_t *chassis_move_control);
//底盘PID计算以及运动分解
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop);

 fp32 motor_speed_pid[3] = {700,0.3,0};
 fp32 motor_angle_pid[3] = {1.9,0,0};
 fp32 M3505_MOTOR_SPEED_PID_MAX_OUT=10000.0f;
fp32 M3505_MOTOR_SPEED_PID_MAX_IOUT=500.0f;
 fp32 CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT=100.0f;
fp32 CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT=0.2f;
 fp32 CHASSIS_WZ_SET_SCALE = 0.0f;
 fp32 CHASSIS_WZ_SET_YOU = 0.0f;
//主任务
void chassis_task(void *pvParameters)
{
    //空闲一段时间
    vTaskDelay(CHASSIS_TASK_INIT_TIME);
    //底盘初始化
    chassis_init(&chassis_move);

    //判断底盘电机是否都在线
    while (1)
    {
        //遥控器设置状态
        chassis_set_mode(&chassis_move);
        //遥控器状态切换数据保存
        chassis_mode_change_control_transit(&chassis_move);
        //底盘数据更新
        chassis_feedback_update(&chassis_move);
        //底盘控制量设置
        chassis_set_contorl(&chassis_move);
        //底盘控制PID计算
        chassis_control_loop(&chassis_move);
				//接收机，陀螺仪断线,YAW轴断线，云台不动
		if(Detect_Judeg(5)||Detect_Judeg(0)||Detect_Judeg(1)||Detect_Judeg(2)||Detect_Judeg(3)||Detect_Judeg(4))
		{
			CAN_cmd_chassis(0,0,0,0);
		}
		else
		{
			CAN_cmd_chassis(chassis_move.motor_chassis[0].give_current, chassis_move.motor_chassis[1].give_current,
                                chassis_move.motor_chassis[2].give_current, chassis_move.motor_chassis[3].give_current);
		}
        //系统延时
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

    //底盘开机状态为停止
    chassis_move_init->chassis_mode = CHASSIS_VECTOR_RAW;//无力
    //获取遥控器指针
    chassis_move_init->chassis_RC = get_remote_control_point();
    //获取陀螺仪姿态角指针
    //获取云台电机数据指针
    chassis_move_init->chassis_yaw_motor = get_Yaw_Gimbal_Motor_Measure_Point();
    //初始化PID 运动
    for (i = 0; i < 4; i++)
    {
        chassis_move_init->motor_chassis[i].chassis_motor_measure = get_Chassis_Motor_Measure_Point(i);
        PID_init(&chassis_move_init->motor_speed_pid[i], PID_POSITION, motor_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
    }
    //初始化旋转PID
    PID_init(&chassis_move_init->chassis_angle_pid, PID_POSITION, motor_angle_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);
    //用一阶滤波代替斜波函数生成
    //最大 最小速度
    chassis_move_init->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
    chassis_move_init->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;

    chassis_move_init->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
    chassis_move_init->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;

    //更新一下数据
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

    //切入跟随云台模式
    if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {
        chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
    }
    //切入不跟随云台模式
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
        //更新电机速度，加速度是速度的PID微分
			chassis_move_update->motor_chassis[i].speed = (chassis_move_update->motor_chassis[i].chassis_motor_measure->speed_rpm*PI/30.0f)/19.0f;//角速度
    }

    //更新底盘前进速度 x， 平移速度y，旋转速度wz，坐标系为右手系

    //计算底盘姿态角度, 如果底盘上有陀螺仪请更改这部分代码
		total_ecd = chassis_move_update->chassis_yaw_motor->ecd_out - 3500;
		chassis_move_update->chassis_yaw = theta_format(total_ecd / (8192.0f/360.0f));
}

//遥控器的数据处理成底盘的前进vx速度，vy速度
void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL)
    {
        return;
    }
    //遥控器原始通道值
    int16_t vx_channel, vy_channel;
    fp32 vx_set_channel, vy_set_channel;
    //死区限制，因为遥控器可能存在差异 摇杆在中间，其值不为0
    rc_deadline_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
    rc_deadline_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE);

			vx_set_channel = (vx_channel * CHASSIS_VX_RC_SEN);
			vy_set_channel = (vy_channel * CHASSIS_VY_RC_SEN);

    //停止信号，不需要缓慢加速，直接减速到零
		if(vx_channel==0&&vy_channel==0)
		{
			*vx_set=*vy_set=0;
		}
		
		*vx_set = vx_set_channel;
    *vy_set = vy_set_channel;

}

//设置遥控器输入控制量
static void chassis_set_contorl(chassis_move_t *chassis_move_control)
{
    
    if (chassis_move_control == NULL)
    {
        return;
    }
    static float d_theta = 0;
    //设置速度
    fp32 vx_set = 0.0f, vy_set = 0.0f, angle_set = 0.0f;
    chassis_behaviour_control_set(&vx_set, &vy_set, &angle_set, chassis_move_control);

    //跟随云台模式
    if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {
        fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
        //旋转控制底盘速度方向，保证前进方向是云台方向，有利于运动平稳
			//旋转控制底盘速度方向，保证前进方向是云台方向，有利于运动平稳
			if(chassis_move_control->chassis_yaw < 0){
				d_theta =  chassis_move_control->chassis_yaw - 0;
			}
			else if(chassis_move_control->chassis_yaw <= 0+ 180 ){
				d_theta =chassis_move_control->chassis_yaw - 0;
			}
			else if(chassis_move_control->chassis_yaw <= 360){
				d_theta = 360 -  chassis_move_control->chassis_yaw + 0;
			}
			d_theta /= -57.295;//角度转弧度
			
			sin_yaw = arm_sin_f32(d_theta);
			cos_yaw = arm_cos_f32(d_theta);

			chassis_move_control->vx_set = cos_yaw * vx_set - sin_yaw * vy_set;
			chassis_move_control->vy_set = sin_yaw * vx_set + cos_yaw * vy_set;
        //设置控制相对云台角度
        chassis_move_control->chassis_relative_angle_set = angle_set;
        //计算旋转PID角速度
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
        //速度限幅
        chassis_move_control->vx_set = fp32_constrain(chassis_move_control->vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(chassis_move_control->vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
    }
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
    {

        //放弃跟随云台
        //这个模式下，角度设置的为 角速度
        fp32 chassis_wz = angle_set;
        chassis_move_control->wz_set = chassis_wz;
        chassis_move_control->vx_set = vx_set;
        chassis_move_control->vy_set = vy_set;
    }
}



static void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4])
{
    //旋转的时候， 由于云台靠前，所以是前面两轮 0 ，1 旋转的速度变慢， 后面两轮 2,3 旋转的速度变快
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
    //麦轮运动分解
    chassis_vector_to_mecanum_wheel_speed(chassis_move_control_loop->vx_set,
                                          chassis_move_control_loop->vy_set, chassis_move_control_loop->wz_set, wheel_speed);

if(chassis_move_control_loop->chassis_mode == CHASSIS_VECTOR_RAW){
for (i = 0; i < 4; i++){
chassis_move_control_loop->motor_chassis[i].give_current = 0;
}
}
else if(chassis_move_control_loop->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW){
    //计算pid

    for (i = 0; i < 4; i++)
    {
			  chassis_move_control_loop->motor_chassis[i].speed_set = wheel_speed[i];
        PID_calc(&chassis_move_control_loop->motor_speed_pid[i], chassis_move_control_loop->motor_chassis[i].speed, chassis_move_control_loop->motor_chassis[i].speed_set);
    }

    //赋值电流值
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
