#include "shoot_task.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "tim.h"
#include "Gimbal_Task.h"
Shoot_DATA SHOOT_TASK;
#define Switich_ON 1
#define Switich_OFF 0
static char Bullet_mode = Switich_OFF;
static int Bullet_Flag = Switich_OFF;
#define MODE_RC_CHANNEL    0  //选择底盘状态 开关通道号
#define SHOOT_RC_CHANNEL   1  //选择发射状态 开关通道号
//右拨杆通道值
#define RC_SWITCH_DOWN 2
#define RC_SWITCH_MID 3
#define RC_SWITCH_UP 1
int SPPED = 500;
int SPPED1 = 500;
void shoot_task(void const * argument)
{
  Shoot_Init(&SHOOT_TASK); 
	
  for(;;)
  {
	Shoot_Mode_Set(&SHOOT_TASK);
		
	Shoot_Updata(&SHOOT_TASK);
		
	Shoot_move(&SHOOT_TASK);

	Shoot_GiveCUR(&SHOOT_TASK);
		
  osDelay(1);
  }
}

Shoot_DATA *Get_Shoot_DATA()
{
	return &SHOOT_TASK;
}
/*****************************************************************************************/
void Shoot_Mode_Set(Shoot_DATA *SHOOT_MODE_SET)
{ 
	SHOOT_MODE_SET->Shoot_Mode.Last_Shoot_mode = SHOOT_MODE_SET->Shoot_Mode.Shoot_mode;
 if(SHOOT_MODE_SET->Get_Rc->rc.s[MODE_RC_CHANNEL] == RC_SWITCH_UP)
  {
		SHOOT_MODE_SET->Shoot_Mode.Use_Mode = Handle;
  }
 if(SHOOT_MODE_SET->Get_Rc->rc.s[MODE_RC_CHANNEL] == RC_SWITCH_MID)
  {
		SHOOT_MODE_SET->Shoot_Mode.Use_Mode = KeyBord;
  }

  if(SHOOT_MODE_SET->Get_Rc->rc.s[SHOOT_RC_CHANNEL] == RC_SWITCH_UP)
  {
	 SHOOT_MODE_SET->Shoot_Mode.Shoot_mode = SHOOT_ONECE;
  }
	
	  if(SHOOT_MODE_SET->Get_Rc->rc.s[SHOOT_RC_CHANNEL] == RC_SWITCH_MID||SHOOT_MODE_SET->Get_Rc->rc.s[SHOOT_RC_CHANNEL] == RC_SWITCH_DOWN)
  {
	 SHOOT_MODE_SET->Shoot_Mode.Shoot_mode = SHOOT_WEAK;
  }
 
//还原拨盘位置，防止无力拨动拨盘后射击模式疯转
	if(SHOOT_MODE_SET->Shoot_Mode.Last_Shoot_mode == SHOOT_WEAK && (SHOOT_MODE_SET->Shoot_Mode.Shoot_mode == SHOOT_ONECE || SHOOT_MODE_SET->Shoot_Mode.Shoot_mode == SHOOT_KEEP
		 || SHOOT_MODE_SET->Get_Rc->rc.s[MODE_RC_CHANNEL] == RC_SWITCH_MID || SHOOT_MODE_SET->Get_Rc->rc.s[MODE_RC_CHANNEL] == RC_SWITCH_UP))
  {
	SHOOT_MODE_SET->Shoot_PID.Shoot_ANGLE_PID.ref = SHOOT_MODE_SET->TRI_MOTOR->total_angle;
  }
 
}


/********************************************************rfffffffffffffffffffffffffffffffffffffffr*********************************/
void Shoot_Init(Shoot_DATA *SHOOT_INIT)
{ 
	SHOOT_INIT->Get_Rc = get_remote_control_point();              //读取遥控器数据
	SHOOT_INIT->TRI_MOTOR = Get_Pluck_Motor_Data_Point();      //读取拨盘数据
	SHOOT_INIT->FRI_MOTOR1 = Get_Fri1_Motor_Data_Point();
	SHOOT_INIT->FRI_MOTOR2 = Get_Fri2_Motor_Data_Point();	     //读取摩擦轮数据
	static fp32 Shoot_SPEED_PID[3] = {20,0,0};
	static fp32 Shoot_ANGLE_PID[3] = {12,0,0};
	static fp32 Fri_SPEED_PID1[3] = {990,0.001,0};
	static fp32 Fri_SPEED_PID2[3] = {990,0.001,0};
  static fp32 Fri_Angle_PID[3] = {20,0,0};


  SPEED_PID_Init(&SHOOT_INIT->Shoot_PID.Shoot_SPEED_PID,Shoot_SPEED_PID,Shoot_MAXOUT,Shoot_MAXIOUT);
  Angle_Pid_Init(&SHOOT_INIT->Shoot_PID.Shoot_ANGLE_PID,Shoot_ANGLE_PID,Shoot_MAXOUT,Shoot_MAXIOUT);
	
  SPEED_PID_Init(&SHOOT_INIT->Shoot_PID.Fri_SPEED_PID1,Fri_SPEED_PID1,Fri_MAXOUT,Shoot_MAXIOUT);
  SPEED_PID_Init(&SHOOT_INIT->Shoot_PID.Fri_SPEED_PID2,Fri_SPEED_PID2,Fri_MAXOUT,Shoot_MAXIOUT);
	Angle_Pid_Init(&SHOOT_INIT->Shoot_PID.Fri_ANGLE_PID1,Fri_Angle_PID,Shoot_MAXOUT,Shoot_MAXIOUT);
	Angle_Pid_Init(&SHOOT_INIT->Shoot_PID.Fri_ANGLE_PID2,Fri_Angle_PID,Shoot_MAXOUT,Shoot_MAXIOUT);
	
	SHOOT_INIT->Shoot_Mode.Shoot_Time =  Tri_ON;
	SHOOT_INIT->Shoot_Mode.KEEP_MODE = KEEP_NO;
	SHOOT_INIT->Shoot_Mode.Shoot_Protect = NOMAL;
	SHOOT_INIT->Shoot_Mode.Shoot_mode = SHOOT_WEAK;
}

/*****************************************************************************************/
void Shoot_Updata(Shoot_DATA *SHOOT_UPDATA)
{
	if(SHOOT_UPDATA->Get_Rc->rc.s[MODE_RC_CHANNEL] != RC_SWITCH_DOWN)
	{
	SHOOT_UPDATA->Shoot_PID.Shoot_SPEED_PID.fdb = SHOOT_UPDATA->TRI_MOTOR->speed_rpm;
	SHOOT_UPDATA->Shoot_PID.Shoot_ANGLE_PID.fdb = SHOOT_UPDATA->TRI_MOTOR->total_angle;	
	}
	
	SHOOT_UPDATA->Shoot_PID.Fri_SPEED_PID1.fdb = SHOOT_UPDATA->FRI_MOTOR1->angular_velocity;
	SHOOT_UPDATA->Shoot_PID.Fri_SPEED_PID2.fdb = SHOOT_UPDATA->FRI_MOTOR2->angular_velocity;
	
	SHOOT_UPDATA->Shoot_PID.Fri_ANGLE_PID1.fdb = SHOOT_UPDATA->FRI_MOTOR1->speed_rpm;
	SHOOT_UPDATA->Shoot_PID.Fri_ANGLE_PID2.fdb = SHOOT_UPDATA->FRI_MOTOR2->speed_rpm;
	
}

/*****************************************************************************************/
void Shoot_move(Shoot_DATA *SHOOT_PID)
{
static int De_Bug_Flag = 0,De_Bug_OnceEnd_Flag; //调试
static long long De_Bug_First_Time,De_Bug_Second_Time;
static long long first_time,second_time,keep_time_ON,keep_time_OFF,stoppage_Time,reverse_Time;
static int KEYBORD_FLAG,stoppage_FLAG;
static fp32 MAX_Shoot_Speed_Limit;
/************************************弹仓*******************************************/
if(SHOOT_PID->Shoot_Mode.Use_Mode == Handle)
{
    Bullet_Flag = Switich_ON;
}
else
{
Bullet_Flag = Switich_OFF;
}


 if(Bullet_Flag == Switich_ON) 
 {
TIM1 -> CCR2 = 4500;//4500 
TIM1 -> CCR3 = 1000;//1000
 }
 if(Bullet_Flag == Switich_OFF) 
 {
TIM1 -> CCR2 = 2000; //2000
TIM1 -> CCR3 = 3200;//3200
 }
	
/***************************摩擦轮*************************************/
//限制射速
MAX_Shoot_Speed_Limit =30;


/****************************拨盘**************************************/
static long compensation_Tick = 0;
//调试单发模式
if(SHOOT_PID->Get_Rc->rc.ch[4]> UP_BULLET_DEADLINE && De_Bug_Flag == 0 && SHOOT_PID->Get_Rc->rc.s[MODE_RC_CHANNEL] != RC_SWITCH_DOWN)
{
	if(SHOOT_PID->Shoot_Mode.Shoot_mode != SHOOT_WEAK && __fabs(SHOOT_PID->FRI_MOTOR1->angular_velocity) > 2 
		                                     && __fabs(SHOOT_PID->FRI_MOTOR2->angular_velocity) > 2)
	{
	De_Bug_Flag = 1;
	SHOOT_PID->Shoot_PID.Shoot_ANGLE_PID.ref += ONECE_TRI_ANGLE;
  compensation_Tick++;		
	}
}

////调试连发模式
if(SHOOT_PID->Get_Rc->rc.ch[4] < Down_BULLET_DEADLINE)
{
	if(De_Bug_OnceEnd_Flag == 0)
	{
		De_Bug_First_Time = xTaskGetTickCount();
		De_Bug_OnceEnd_Flag = 1;
	}
	De_Bug_Second_Time = xTaskGetTickCount();
	if((De_Bug_Second_Time - De_Bug_First_Time) > (1000/Shoot_Kaluli_limit(SHOOT_PID)) && __fabs(SHOOT_PID->FRI_MOTOR1->angular_velocity) > 2 
		                                     && __fabs(SHOOT_PID->FRI_MOTOR2->angular_velocity) > 2)
	{
		SHOOT_PID->Shoot_PID.Shoot_ANGLE_PID.ref += ONECE_TRI_ANGLE;
		compensation_Tick++;
		De_Bug_OnceEnd_Flag = 0;
	}
  SHOOT_PID->Shoot_Can2_Send.Shoot_More_Flag = 1;
}
else
{
	SHOOT_PID->Shoot_Can2_Send.Shoot_More_Flag = 0;
}

//标志位回位
if(__fabs(SHOOT_PID->Get_Rc->rc.ch[4]) < BULLET_DEADLINE1)
{
		De_Bug_Flag = 0;
	  De_Bug_OnceEnd_Flag = 0;
}
//键盘

//遥控器
 /**********************************PID计算**************************************/

	Angle_Pid_Cala(&SHOOT_PID->Shoot_PID.Shoot_ANGLE_PID,SHOOT_PID->Shoot_PID.Shoot_ANGLE_PID.fdb,SHOOT_PID->Shoot_PID.Shoot_ANGLE_PID.ref);

	SHOOT_PID->Shoot_PID.Shoot_SPEED_PID.ref = SHOOT_PID->Shoot_PID.Shoot_ANGLE_PID.out;
	Speed_Pid_Calc(&SHOOT_PID->Shoot_PID.Shoot_SPEED_PID);

	if(SHOOT_PID->Shoot_Mode.Shoot_mode == SHOOT_KEEP || SHOOT_PID->Shoot_Mode.Shoot_mode == SHOOT_ONECE)
{
	SHOOT_PID->Shoot_PID.Fri_SPEED_PID1.ref = MAX_Shoot_Speed_Limit;
  SHOOT_PID->Shoot_PID.Fri_SPEED_PID2.ref = -MAX_Shoot_Speed_Limit;
	Speed_Pid_Calc(&SHOOT_PID->Shoot_PID.Fri_SPEED_PID1);
	Speed_Pid_Calc(&SHOOT_PID->Shoot_PID.Fri_SPEED_PID2);
}
 if(SHOOT_PID->Shoot_Mode.Shoot_mode == SHOOT_WEAK)
 {
	SHOOT_PID->Shoot_Mode.Shoot_Protect = NOMAL;
	SHOOT_PID->Shoot_PID.Fri_ANGLE_PID1.ref = FRI_STOP_ANGLE_REF;
	SHOOT_PID->Shoot_PID.Fri_ANGLE_PID2.ref = FRI_STOP_ANGLE_REF;
	Angle_Pid_Cala(&SHOOT_PID->Shoot_PID.Fri_ANGLE_PID1,SHOOT_PID->Shoot_PID.Fri_ANGLE_PID1.fdb,SHOOT_PID->Shoot_PID.Fri_ANGLE_PID1.ref);
	Angle_Pid_Cala(&SHOOT_PID->Shoot_PID.Fri_ANGLE_PID2,SHOOT_PID->Shoot_PID.Fri_ANGLE_PID2.fdb,SHOOT_PID->Shoot_PID.Fri_ANGLE_PID2.ref);
	SHOOT_PID->Shoot_PID.Fri_SPEED_PID1.out = SHOOT_PID->Shoot_PID.Fri_ANGLE_PID1.out;
	SHOOT_PID->Shoot_PID.Fri_SPEED_PID2.out = SHOOT_PID->Shoot_PID.Fri_ANGLE_PID2.out;
	//清除PID参数，防止上一次的参数影响下一次
  angle_PID_clear(&SHOOT_PID->Shoot_PID.Shoot_ANGLE_PID);
 }
if(SHOOT_PID->Get_Rc->rc.s[MODE_RC_CHANNEL] == RC_SWITCH_DOWN)
   SHOOT_PID->Shoot_PID.Shoot_SPEED_PID.out = SHOOT_SPEED_STOP_OUT;

}
/*****************************************************************************************/
void Shoot_GiveCUR(Shoot_DATA *SHOOT_CUR)
{
	static int16_t Count_Fri1,Count_Fri2;
 	Count_Fri1 = SHOOT_CUR->Shoot_PID.Fri_SPEED_PID1.out;
	Count_Fri2 = SHOOT_CUR->Shoot_PID.Fri_SPEED_PID2.out;
	
	send_shoot_cur(Count_Fri1,Count_Fri2,SHOOT_CUR->Shoot_PID.Shoot_SPEED_PID.out);
}
/*****************************************************************************************/
 float Shoot_Kaluli_limit(Shoot_DATA *SHOOT_heat_limt)     
 {
	
	static float E_Shoot_Rate;
	 
	 E_Shoot_Rate = 15;
	 return E_Shoot_Rate;           //返回射频
 }

