#include "BMI088_Drive.h"
#include "BMI088.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "Get_BMI088_Data.h"
#include "math.h"
#include "MahonyAHRS.h"
#include "IST8310.h"
#include "Imu_temp_control.h"
//#include "Gimbal_Task.h"
//#include "pid.h"
	static GYRO_INFO 	*GYRO_DATA;	
	static ACC_INFO *ACC_DATA;
	static IST8310_INFO * IST8310_DATA;
	GYRO_INFO Base_Gyro;
	ACC_INFO Base_Acc;
	static IST8310_INFO Base_IST8310;
	extern  Pid_type_def imu_temp_pid;

	GYRO_INFO *GET_GYRO_DATA()
{
	return &Base_Gyro;
};

ACC_INFO *GET_ACC_DATA()
{
	return &Base_Acc;
};
IST8310_INFO *GETIST8310_address()
{
	return &Base_IST8310;
};

/*加速度的解算*/

void Acc_Solution(void)
{	    static int32_t yaw_connt=0;
	   	static float pitch_angle,yaw_angle,last_yaw_angle=0;
			float Q123[4];
			MahonyAHRSupdateIMU(GYRO_DATA->v_x, GYRO_DATA->v_y,GYRO_DATA->v_z, ACC_DATA->acc_x, ACC_DATA->acc_y, ACC_DATA->acc_z);
			Q123[0]=q0;
			Q123[1]=q1;
			Q123[2]=q2;
			Q123[3]=q3;
			get_angle(Q123,&GYRO_DATA->ANGLE_YAW,&GYRO_DATA->ANGLE_PIT,&GYRO_DATA->ANGLE_ROW);
      Base_Gyro.use_v_x = GYRO_DATA->v_x*57.295;
			Base_Gyro.use_v_y = GYRO_DATA->v_y*57.295;
			Base_Gyro.use_v_z = GYRO_DATA->v_z*57.295;
	
	    	if((GYRO_DATA->ANGLE_YAW -last_yaw_angle) > 330)
			 {
				yaw_connt--;
			 }
			 else if((GYRO_DATA->ANGLE_YAW - last_yaw_angle) < -330)
			 {
				yaw_connt++;
			 }
			  Base_Gyro.GET_YAW = GYRO_DATA->ANGLE_YAW+ yaw_connt * 360.0f;	 
				last_yaw_angle = GYRO_DATA->ANGLE_YAW;
			 
			  Base_Gyro.GET_PIT = GYRO_DATA->ANGLE_PIT;
			  Base_Gyro.GET_ROW = GYRO_DATA->ANGLE_ROW;
//			 if(GIMBAL_COUNT->Use_Flag.gimbal_state == Gimbal_Weak)
//			 {
//				 yaw_connt = 0;
//			 }
			 
}
/*Y、P、R轴角度解算*/
void get_angle(float *q, float *yaw, float *pitch, float *roll)
{
	float sqw=q[0] * q[0];
	float sqx=q[1] * q[1];
	float sqy=q[2] * q[2];
	float sqz=q[3] * q[3];
		
	*yaw = atan2f(2.0f*(q[0]*q[3]+q[1]*q[2]), 2.0f*(q[0]*q[0]+q[1]*q[1])-1.0f)*180.0f/PI;
	*roll = asinf(-2.0f*(q[1]*q[3]-q[0]*q[2]))*180.0f/PI;
	*pitch = atan2f(2.0f*(q[0]*q[1]+q[2]*q[3]),2.0f*(q[0]*q[0]+q[3]*q[3])-1.0f)*180.0f/PI;

}

/*该函数用于调整误差，BMI088、IST8310、温控PID的初始化*/
void Init_offset_and_gyro(double V_X_offset,double V_Y_offset,double V_Z_offset,double ACC_ratio)
{	
	static const float imu_temp_PID[3] = {TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD};
	imu_temp_PID_init(&imu_temp_pid, PID_POSITION, imu_temp_PID, TEMPERATURE_PID_MAX_OUT, TEMPERATURE_PID_MAX_IOUT);
	IST8310_IIC_WHO_AM_I();//磁力计
	BMI088_Init();
	GYRO_DATA = GET_GYRO_DATA();
	ACC_DATA = GET_ACC_DATA();
	IST8310_DATA = GETIST8310_address();
	GYRO_DATA->v_x_offset = V_X_offset;//-3.41799998;
	GYRO_DATA->v_y_offset = V_Y_offset;//-1.82200003;
	GYRO_DATA->v_z_offset = V_Z_offset;//-3.398;
	ACC_DATA->acc_ratio =  ACC_ratio;//0.9908096240504;
}
/*放到任务循环里面，用于读取陀螺仪，磁力计，云台Y,P,R轴数据和温度控制*/
void Loop_Get_Gyro_Data()
{
		get_imu_data();
		imu_temp_control(ACC_DATA->temperature);
		Acc_Solution();
}

void INS_task(void const * argument)
{
  Init_offset_and_gyro(-2.15133333,-3.98033333,-1.80700004,0.9942550635784);
  
  for(;;)
  {
    Loop_Get_Gyro_Data(); 
//		calibrate_imu(3000);
    osDelay(1);
  }


}