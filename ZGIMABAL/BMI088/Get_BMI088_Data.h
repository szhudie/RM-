#ifndef Get_BMI088_Data_H
#define Get_BMI088_Data_H

#include "main.h"
#define PI                 3.14159265358979f




typedef struct
{
	int8_t get_IST8310_Data[6];
	
	int16_t get_IST8310_x;
	int16_t get_IST8310_y;
	int16_t get_IST8310_z;
	
	float IST8310_x;
	float IST8310_y;
	float IST8310_z;

	
}IST8310_INFO;

typedef struct
{
	int16_t get_speed_Vz;
	int16_t get_speed_Vy;	
	int16_t get_speed_Vx;
	
	float v_z;
	float v_y;	
	float v_x;

	float ANGLE_PIT;
	float ANGLE_YAW;
	float ANGLE_ROW;
	
	float v_z_offset;
	float v_y_offset;	
	float v_x_offset;
	
}GYRO_DATA_t;

typedef struct
{
	int16_t get_speed_Vz;
	int16_t get_speed_Vy;	
	int16_t get_speed_Vx;
	
	float v_z;
	float v_y;	
	float v_x;

	float ANGLE_PIT;
	float ANGLE_YAW;
	float ANGLE_ROW;
	
	float v_z_offset;
	float v_y_offset;	
	float v_x_offset;
	
	float GET_YAW;
	float GET_PIT;
	float GET_ROW;
	
	float use_v_z;
	float use_v_y;	
	float use_v_x;
	
	float offset_ANGLE_YAW;
	float offset_ANGLE_PIT;
}GYRO_INFO;



typedef struct
{
	 int16_t get_ACC_z;
	 int16_t get_ACC_y;	
	 int16_t get_ACC_x;
	 int16_t get_temp;
	
	 float acc_z;
	 float acc_y;	
	 float acc_x;
	 float	temperature;
	double acc_ratio;
}ACC_INFO;





GYRO_INFO *GET_GYRO_DATA(void);
ACC_INFO *GET_ACC_DATA(void);
IST8310_INFO *GETIST8310_address(void);
void get_angle(float *q, float *yaw, float *pitch, float *roll);
void imu_pwm_set(uint16_t pwm);
void Acc_Solution(void);
void Init_offset_and_gyro(double V_X_offset,double V_Y_offset,double V_Z_offset,double ACC_ratio);
void Loop_Get_Gyro_Data(void);




#endif

