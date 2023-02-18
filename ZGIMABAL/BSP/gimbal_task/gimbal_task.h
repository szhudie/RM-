/**
  * @file       gimbal_task.c/h
  * @brief      �����̨��������������̨ʹ�������ǽ�����ĽǶȣ��䷶Χ�ڣ�-pi,pi��
  *             �ʶ�����Ŀ��ǶȾ�Ϊ��Χ���������ԽǶȼ���ĺ�������̨��Ҫ��Ϊ2��
  *             ״̬�������ǿ���״̬�����ð��������ǽ������̬�ǽ��п��ƣ�����������
  *             ״̬��ͨ����������ı���ֵ���Ƶ�У׼�����⻹��У׼״̬��ֹͣ״̬�ȡ�
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

//pitch �ٶȻ� PID�����Լ� PID���������������
#define PITCH_SPEED_PID_MAX_OUT 30000.0f
#define PITCH_SPEED_PID_MAX_IOUT 2000.0f

//yaw �ٶȻ� PID�����Լ� PID���������������
#define YAW_SPEED_PID_MAX_OUT 30000.0f
#define YAW_SPEED_PID_MAX_IOUT 2000.0f

//pitch �ǶȻ� �Ƕ��������ǽ��� PID�����Լ� PID���������������
#define PITCH_GYRO_ABSOLUTE_PID_KP 5.0f
#define PITCH_GYRO_ABSOLUTE_PID_KI 0.0f
#define PITCH_GYRO_ABSOLUTE_PID_KD 0.3f

#define PITCH_GYRO_ABSOLUTE_PID_MAX_OUT 200.0f
#define PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT 30.0f

//yaw �ǶȻ� �Ƕ��������ǽ��� PID�����Լ� PID���������������
//#define YAW_GYRO_ABSOLUTE_PID_KP 1.195f
//#define YAW_GYRO_ABSOLUTE_PID_KI 0.0f
//#define YAW_GYRO_ABSOLUTE_PID_KD 0.1f
#define YAW_GYRO_ABSOLUTE_PID_MAX_OUT 650.0f
#define YAW_GYRO_ABSOLUTE_PID_MAX_IOUT 100.0f

//pitch �ǶȻ� �Ƕ��ɱ����� PID�����Լ� PID���������������
#define PITCH_ENCODE_RELATIVE_PID_KP 15.0f
#define PITCH_ENCODE_RELATIVE_PID_KI 0.00f
#define PITCH_ENCODE_RELATIVE_PID_KD 0.0f

#define PITCH_ENCODE_RELATIVE_PID_MAX_OUT 0.0f
#define PITCH_ENCODE_RELATIVE_PID_MAX_IOUT 0.0f

//yaw �ǶȻ� �Ƕ��ɱ����� PID�����Լ� PID���������������
#define YAW_ENCODE_RELATIVE_PID_KP 1.195f
#define YAW_ENCODE_RELATIVE_PID_KI 0.0f
#define YAW_ENCODE_RELATIVE_PID_KD 0.0f
#define YAW_ENCODE_RELATIVE_PID_MAX_OUT 100.0f
#define YAW_ENCODE_RELATIVE_PID_MAX_IOUT 0.0f

//yaw,pitch����ͨ���Լ�״̬����ͨ��
#define YawChannel 2
#define PitchChannel 3
#define ModeChannel 0

//ң����������������Ϊң�������ڲ��죬ҡ�����м䣬��ֵ��һ��Ϊ��
#define RC_deadband 10
//��̨��������
#define GIMBAL_CONTROL_TIME 1

//��̨����ģʽ �궨�� 0 Ϊ��ʹ�ò���ģʽ
#define GIMBAL_TEST_MODE 0

//����Ƿ�װ
#define PITCH_TURN 0
#define YAW_TURN 1

//�������ֵ����Լ���ֵ
#define Half_ecd_range 4096
#define ecd_range 8191
//��̨��ʼ������ֵ����������,��������Χ��ֹͣһ��ʱ���Լ����ʱ��6s������ʼ��״̬��
#define GIMBAL_INIT_ANGLE_ERROR 0.1f
#define GIMBAL_INIT_STOP_TIME 100
#define GIMBAL_INIT_TIME 6000
//��̨��ʼ������ֵ���ٶ��Լ����Ƶ��ĽǶ�
#define GIMBAL_INIT_PITCH_SPEED 0.004f
#define GIMBAL_INIT_YAW_SPEED   0.005f
#define INIT_YAW_SET 0.0f
#define INIT_PITCH_SET 0.0f

//��̨У׼��ֵ��ʱ�򣬷���ԭʼ����ֵ���Լ���תʱ�䣬ͨ���������ж϶�ת
#define GIMBAL_CALI_MOTOR_SET 8000
#define GIMBAL_CALI_STEP_TIME 2000
#define GIMBAL_CALI_GYRO_LIMIT 0.1f

#define GIMBAL_CALI_PITCH_MAX_STEP 1
#define GIMBAL_CALI_PITCH_MIN_STEP 2
#define GIMBAL_CALI_YAW_MAX_STEP 3
#define GIMBAL_CALI_YAW_MIN_STEP 4

#define GIMBAL_CALI_START_STEP GIMBAL_CALI_PITCH_MAX_STEP
#define GIMBAL_CALI_END_STEP 5

//�ж�ң�����������ʱ���Լ�ң�����������жϣ�������̨yaw����ֵ�Է�������Ư��
#define GIMBAL_MOTIONLESS_RC_DEADLINE 10
#define GIMBAL_MOTIONLESS_TIME_MAX 3000

//�������ֵת���ɽǶ�ֵ
#ifndef Motor_Ecd_to_Rad
#define Motor_Ecd_to_Rad 0.000766990394f //      2*  PI  /8192
#endif

typedef enum
{
    GIMBAL_MOTOR_RAW = 0, //���ԭʼֵ����
    GIMBAL_MOTOR_GYRO,    //��������ǽǶȿ���
    GIMBAL_MOTOR_ENCONDE, //�������ֵ�Ƕȿ���
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
	  fp32 Dbuf[3];  //΢���� 0���� 1��һ�� 2���ϴ�
    fp32 error[3]; //����� 0���� 1��һ�� 2���ϴ�

    fp32 out;
	  fp32 last_out;
} Gimbal_PID_t;

typedef struct
{
    const motor_measure_t *gimbal_motor_measure;
    Gimbal_PID_t gimbal_motor_absolute_angle_pid;//���ԽǶ�PID
    Gimbal_PID_t gimbal_motor_relative_angle_pid;//��ԽǶ�PID
	  FUZZY_PID Fuzzy_Pid;

    pid_type_def gimbal_motor_gyro_pid;
    gimbal_motor_mode_e gimbal_motor_mode;//�������ģʽ
    gimbal_motor_mode_e last_gimbal_motor_mode;//��һ���������ģʽ
    uint16_t offset_ecd;   //����ֵ
	
    fp32 max_relative_angle; //�����ԽǶ�
    fp32 min_relative_angle; //��С��ԽǶ�

    fp32 relative_angle;     //��ǰֵ
    fp32 relative_angle_set; //�趨ֵ
	
    fp32 absolute_angle;     //rad
    fp32 absolute_angle_set; //rad
	
    fp32 motor_gyro; //�����ǵ��ٶ�        //rad/s
    fp32 motor_speed; //����ٶ�
	  fp32 motor_gyro_set;//�ǶȻ�����ֵ,�ٶȻ����趨ֵ
	
    
    fp32 raw_cmd_current;
		
    fp32 current_set; //�ٶȻ�����ֵ��ֱ�����ֵ
    int16_t given_current;//����PID����Ľ��,��ʼ������ֵ��raw_cmd_current

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
    const RC_Ctrl_t *gimbal_rc_ctrl;//ң�����ṹ��
    const fp32 *gimbal_INT_angle_point;
    GYRO_INFO *gimbal_INT_gyro_point;
    Gimbal_Motor_t gimbal_yaw_motor;//Y��ṹ��
    Gimbal_Motor_t gimbal_pitch_motor;//P��ṹ��
    Gimbal_Cali_t gimbal_cali;
} Gimbal_Control_t;
extern Gimbal_Control_t gimbal_control;
typedef enum
{
  GIMBAL_ZERO_FORCE = 0, //��̨����
  GIMBAL_INIT,           //��̨��ʼ��
  GIMBAL_CALI,           //��̨У׼
  GIMBAL_ABSOLUTE_ANGLE, //��̨�����Ǿ��ԽǶȿ���
  GIMBAL_RELATIVE_ANGLE, //��̨�������ֵ��ԽǶȿ���
  GIMBAL_MOTIONLESS,     //��̨��ң����������һ��ʱ��󱣳ֲ���������������Ư��
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
