/**
  * @file       gimbal_task.c/h
  * @brief      完成云台控制任务，由于云台使用陀螺仪解算出的角度，其范围在（-pi,pi）
  *             故而设置目标角度均为范围，存在许多对角度计算的函数。云台主要分为2种
  *             状态，陀螺仪控制状态是利用板载陀螺仪解算的姿态角进行控制，编码器控制
  *             状态是通过电机反馈的编码值控制的校准，此外还有校准状态，停止状态等。
  * @note       
  * @history
  */

#ifndef GIMBALTASK_H
#define GIMBALTASK_H
#include "main.h"
#include "bsp_can.h"
#include "pid.h"
#include "bsp_dma.h"
#include "Get_BMI088_Data.h"

//pitch 速度环 PID参数以及 PID最大输出，积分输出
#define PITCH_SPEED_PID_MAX_OUT 30000.0f
#define PITCH_SPEED_PID_MAX_IOUT 2000.0f

//yaw 速度环 PID参数以及 PID最大输出，积分输出
#define YAW_SPEED_PID_MAX_OUT 30000.0f
#define YAW_SPEED_PID_MAX_IOUT 2000.0f

//pitch 角度环 角度由陀螺仪解算 PID参数以及 PID最大输出，积分输出
#define PITCH_GYRO_ABSOLUTE_PID_KP 5.0f
#define PITCH_GYRO_ABSOLUTE_PID_KI 0.0f
#define PITCH_GYRO_ABSOLUTE_PID_KD 0.3f

#define PITCH_GYRO_ABSOLUTE_PID_MAX_OUT 200.0f
#define PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT 30.0f

//yaw 角度环 角度由陀螺仪解算 PID参数以及 PID最大输出，积分输出
//#define YAW_GYRO_ABSOLUTE_PID_KP 1.195f
//#define YAW_GYRO_ABSOLUTE_PID_KI 0.0f
//#define YAW_GYRO_ABSOLUTE_PID_KD 0.1f
#define YAW_GYRO_ABSOLUTE_PID_MAX_OUT 650.0f
#define YAW_GYRO_ABSOLUTE_PID_MAX_IOUT 100.0f

//pitch 角度环 角度由编码器 PID参数以及 PID最大输出，积分输出
#define PITCH_ENCODE_RELATIVE_PID_KP 15.0f
#define PITCH_ENCODE_RELATIVE_PID_KI 0.00f
#define PITCH_ENCODE_RELATIVE_PID_KD 0.0f

#define PITCH_ENCODE_RELATIVE_PID_MAX_OUT 0.0f
#define PITCH_ENCODE_RELATIVE_PID_MAX_IOUT 0.0f

//yaw 角度环 角度由编码器 PID参数以及 PID最大输出，积分输出
#define YAW_ENCODE_RELATIVE_PID_KP 1.195f
#define YAW_ENCODE_RELATIVE_PID_KI 0.0f
#define YAW_ENCODE_RELATIVE_PID_KD 0.0f
#define YAW_ENCODE_RELATIVE_PID_MAX_OUT 100.0f
#define YAW_ENCODE_RELATIVE_PID_MAX_IOUT 0.0f

//yaw,pitch控制通道以及状态开关通道
#define YawChannel 2
#define PitchChannel 3
#define ModeChannel 0

//遥控器输入死区，因为遥控器存在差异，摇杆在中间，其值不一定为零
#define RC_deadband 10
//云台控制周期
#define GIMBAL_CONTROL_TIME 1

//云台测试模式 宏定义 0 为不使用测试模式
#define GIMBAL_TEST_MODE 0

//电机是否反装
#define PITCH_TURN 0
#define YAW_TURN 1

//电机码盘值最大以及中值
#define Half_ecd_range 4096
#define ecd_range 8191
//云台初始化回中值，允许的误差,并且在误差范围内停止一段时间以及最大时间6s后解除初始化状态，
#define GIMBAL_INIT_ANGLE_ERROR 0.1f
#define GIMBAL_INIT_STOP_TIME 100
#define GIMBAL_INIT_TIME 6000
//云台初始化回中值的速度以及控制到的角度
#define GIMBAL_INIT_PITCH_SPEED 0.004f
#define GIMBAL_INIT_YAW_SPEED   0.005f
#define INIT_YAW_SET 0.0f
#define INIT_PITCH_SET 0.0f

//云台校准中值的时候，发送原始电流值，以及堵转时间，通过陀螺仪判断堵转
#define GIMBAL_CALI_MOTOR_SET 8000
#define GIMBAL_CALI_STEP_TIME 2000
#define GIMBAL_CALI_GYRO_LIMIT 0.1f

#define GIMBAL_CALI_PITCH_MAX_STEP 1
#define GIMBAL_CALI_PITCH_MIN_STEP 2
#define GIMBAL_CALI_YAW_MAX_STEP 3
#define GIMBAL_CALI_YAW_MIN_STEP 4

#define GIMBAL_CALI_START_STEP GIMBAL_CALI_PITCH_MAX_STEP
#define GIMBAL_CALI_END_STEP 5

//判断遥控器无输入的时间以及遥控器无输入判断，设置云台yaw回中值以防陀螺仪漂移
#define GIMBAL_MOTIONLESS_RC_DEADLINE 10
#define GIMBAL_MOTIONLESS_TIME_MAX 3000

//电机编码值转化成角度值
#ifndef Motor_Ecd_to_Rad
#define Motor_Ecd_to_Rad 0.000766990394f //      2*  PI  /8192
#endif

typedef enum
{
    GIMBAL_MOTOR_RAW = 0, //电机原始值控制
    GIMBAL_MOTOR_GYRO,    //电机陀螺仪角度控制
    GIMBAL_MOTOR_ENCONDE, //电机编码值角度控制
} gimbal_motor_mode_e;

typedef struct
{
    fp32 kp;
    fp32 ki;
    fp32 kd;

    fp32 set;
    fp32 get;
    fp32 err;

    fp32 max_out;
    fp32 max_iout;

    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
	  fp32 Dout_last;
	  fp32 RC_DF;
	  fp32 Dbuf[3];  //微分项 0最新 1上一次 2上上次
    fp32 error[3]; //误差项 0最新 1上一次 2上上次

    fp32 out;
	  fp32 last_out;
} Gimbal_PID_t;

typedef struct
{
    const motor_measure_t *gimbal_motor_measure;
    Gimbal_PID_t gimbal_motor_absolute_angle_pid;//绝对角度PID
    Gimbal_PID_t gimbal_motor_relative_angle_pid;//相对角度PID
	  FUZZY_PID Fuzzy_Pid;

    pid_type_def gimbal_motor_gyro_pid;
    gimbal_motor_mode_e gimbal_motor_mode;//电机控制模式
    gimbal_motor_mode_e last_gimbal_motor_mode;//上一个电机控制模式
    uint16_t offset_ecd;   //设置值
	
    fp32 max_relative_angle; //最大相对角度
    fp32 min_relative_angle; //最小相对角度

    fp32 relative_angle;     //当前值
    fp32 relative_angle_set; //设定值
	
    fp32 absolute_angle;     //rad
    fp32 absolute_angle_set; //rad
	
    fp32 motor_gyro; //陀螺仪的速度        //rad/s
    fp32 motor_speed; //电机速度
	  fp32 motor_gyro_set;//角度环计算值,速度环的设定值
	
    
    fp32 raw_cmd_current;
		
    fp32 current_set; //速度环计算值、直接输出值
    int16_t given_current;//最终PID计算的结果,开始把他赋值给raw_cmd_current

} Gimbal_Motor_t;

typedef struct
{
    fp32 max_yaw;
    fp32 min_yaw;
    fp32 max_pitch;
    fp32 min_pitch;
    uint16_t max_yaw_ecd;
    uint16_t min_yaw_ecd;
    uint16_t max_pitch_ecd;
    uint16_t min_pitch_ecd;
    uint8_t step;
} Gimbal_Cali_t;


typedef struct
{
    const RC_Ctrl_t *gimbal_rc_ctrl;//遥控器结构体
    const fp32 *gimbal_INT_angle_point;
    GYRO_INFO *gimbal_INT_gyro_point;
    Gimbal_Motor_t gimbal_yaw_motor;//Y轴结构体
    Gimbal_Motor_t gimbal_pitch_motor;//P轴结构体
    Gimbal_Cali_t gimbal_cali;
} Gimbal_Control_t;
extern Gimbal_Control_t gimbal_control;
typedef enum
{
  GIMBAL_ZERO_FORCE = 0, //云台无力
  GIMBAL_INIT,           //云台初始化
  GIMBAL_CALI,           //云台校准
  GIMBAL_ABSOLUTE_ANGLE, //云台陀螺仪绝对角度控制
  GIMBAL_RELATIVE_ANGLE, //云台电机编码值相对角度控制
  GIMBAL_MOTIONLESS,     //云台在遥控器无输入一段时间后保持不动，避免陀螺仪漂移
} gimbal_behaviour_e;


//extern const Gimbal_Motor_t *get_yaw_motor_point(void);
//extern const Gimbal_Motor_t *get_pitch_motor_point(void);
//extern void GIMBAL_task(void *pvParameters);
//extern bool_t cmd_cali_gimbal_hook(uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch);
//extern void set_cali_gimbal_hook(const uint16_t yaw_offset, const uint16_t pitch_offset, const fp32 max_yaw, const fp32 min_yaw, const fp32 max_pitch, const fp32 min_pitch);
extern void GIMBAL_Init(Gimbal_Control_t *gimbal_init);
extern void GIMBAL_PID_Init(Gimbal_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd);
extern void gimbal_behaviour_mode_set(Gimbal_Control_t *gimbal_mode_set);
extern void gimbal_behavour_set(Gimbal_Control_t *gimbal_mode_set);
#endif
