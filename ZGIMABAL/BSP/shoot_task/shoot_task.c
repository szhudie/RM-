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
#define MODE_RC_CHANNEL    0  //ѡ�����״̬ ����ͨ����
#define SHOOT_RC_CHANNEL   1  //ѡ����״̬ ����ͨ����
//�Ҳ���ͨ��ֵ
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
if(SHOOT_MODE_SET->Get_Rc->rc.s[MODE_RC_CHANNEL] == RC_SWITCH_UP)
{
		SHOOT_MODE_SET->Shoot_Mode.Use_Mode = Handle;
}
if(SHOOT_MODE_SET->Get_Rc->rc.s[MODE_RC_CHANNEL] == RC_SWITCH_MID)
{
		SHOOT_MODE_SET->Shoot_Mode.Use_Mode = KeyBord;
}

  if(SHOOT_MODE_SET->Get_Rc->rc.s[SHOOT_RC_CHANNEL] == RC_SWITCH_MID)
 {
	SHOOT_MODE_SET->Shoot_Mode.Shoot_mode = SHOOT_ONECE;
 }
 
//��ԭ����λ�ã���ֹ�����������̺����ģʽ��ת
	if(SHOOT_MODE_SET->Shoot_Mode.Last_Shoot_mode == SHOOT_WEAK && (SHOOT_MODE_SET->Shoot_Mode.Shoot_mode == SHOOT_ONECE || SHOOT_MODE_SET->Shoot_Mode.Shoot_mode == SHOOT_KEEP
		 || SHOOT_MODE_SET->Get_Rc->rc.s[MODE_RC_CHANNEL] == RC_SWITCH_MID || SHOOT_MODE_SET->Get_Rc->rc.s[MODE_RC_CHANNEL] == RC_SWITCH_UP))
  {
	SHOOT_MODE_SET->Shoot_PID.Shoot_ANGLE_PID.ref = SHOOT_MODE_SET->TRI_MOTOR->total_angle;
  }
 
//���ģʽ����
  if(SHOOT_MODE_SET->Shoot_Mode.Last_Shoot_mode == SHOOT_ONECE &&SHOOT_MODE_SET->Get_Rc->rc.s[SHOOT_RC_CHANNEL] == RC_SWITCH_UP&&SHOOT_MODE_SET->Shoot_Mode.Shoot_Protect!=STOPPAGE)
  {
//	SHOOT_MODE_SET->Shoot_PID.Shoot_ANGLE_PID.ref += ONECE_TRI_ANGLE;
	SHOOT_MODE_SET->Shoot_Mode.Last_Shoot_mode = SHOOT_ONECE;
  }
	
	if(SHOOT_MODE_SET->Get_Rc->rc.s[SHOOT_RC_CHANNEL] == RC_SWITCH_UP)
 {
	SHOOT_MODE_SET->Shoot_Mode.Shoot_mode = SHOOT_KEEP;
	 SHOOT_MODE_SET->Shoot_Mode.Last_Shoot_mode = SHOOT_KEEP;
 }
 
	if(SHOOT_MODE_SET->Get_Rc->rc.s[SHOOT_RC_CHANNEL] == RC_SWITCH_MID && SHOOT_MODE_SET->Shoot_Mode.Last_Shoot_mode == SHOOT_KEEP)
 {
	SHOOT_MODE_SET->Shoot_Mode.Shoot_mode = SHOOT_KEEP;
	SHOOT_MODE_SET->Shoot_Mode.Last_Shoot_mode = SHOOT_ONECE;
 }
 
 	if(SHOOT_MODE_SET->Get_Rc->rc.s[SHOOT_RC_CHANNEL] == RC_SWITCH_MID && SHOOT_MODE_SET->Shoot_Mode.Last_Shoot_mode == SHOOT_WEAK)
 {
	SHOOT_MODE_SET->Shoot_Mode.Last_Shoot_mode = SHOOT_ONECE;
 }
 
//����ģʽ����
//if(SHOOT_MODE_SET->Shoot_Mode.Use_Mode == KEYBORAD)
//{ 
//	if(SHOOT_MODE_SET->Get_Rc->Mouse.press_r == 1)
//	{
//		SHOOT_MODE_SET->Shoot_Mode.Shoot_mode = SHOOT_AIM;
//	}
//}

//�������
  if((SHOOT_MODE_SET->Get_Rc->rc.s[SHOOT_RC_CHANNEL] == RC_SWITCH_DOWN && SHOOT_MODE_SET->Shoot_Mode.Shoot_mode != SHOOT_AIM 
	  	&&SHOOT_MODE_SET->Shoot_Mode.Shoot_mode != SHOOT_BUFF)
		  ||SHOOT_MODE_SET->Get_Rc->rc.s[MODE_RC_CHANNEL] == RC_SWITCH_DOWN)
{
	SHOOT_MODE_SET->Shoot_Mode.Shoot_mode = SHOOT_WEAK;
	SHOOT_MODE_SET->Shoot_Mode.Last_Shoot_mode = SHOOT_WEAK;
}
}


/********************************************************rfffffffffffffffffffffffffffffffffffffffr*********************************/
void Shoot_Init(Shoot_DATA *SHOOT_INIT)
{ 
	SHOOT_INIT->Get_Rc = get_remote_control_point();              //��ȡң��������
	SHOOT_INIT->TRI_MOTOR = Get_Pluck_Motor_Data_Point();      //��ȡ��������
	SHOOT_INIT->FRI_MOTOR1 = Get_Fri1_Motor_Data_Point();
	SHOOT_INIT->FRI_MOTOR2 = Get_Fri2_Motor_Data_Point();	     //��ȡĦ��������
//	SHOOT_INIT->CAN2_RC_DATA = Get_CAN2_Referee_Data_Point();  //��ȡCAN2����
//	SHOOT_INIT->gk_DATA = GET_GK_DATA();                       //��ȡ��������
//	SHOOT_INIT->Get_Gimbal_Data = GET_GIMBAL_DATA();           //��ȡ��̨����
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
static int De_Bug_Flag = 0,De_Bug_OnceEnd_Flag; //����
static long long De_Bug_First_Time,De_Bug_Second_Time;
static long long first_time,second_time,keep_time_ON,keep_time_OFF,stoppage_Time,reverse_Time;
static int KEYBORD_FLAG,stoppage_FLAG;
static fp32 MAX_Shoot_Speed_Limit;
/************************************����*******************************************/
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
	
/***************************Ħ����*************************************/
//��������
MAX_Shoot_Speed_Limit =30;
//if(SHOOT_PID->CAN2_RC_DATA->shooter_id1_17mm_speed_limit == 15)
//{
//	MAX_Shoot_Speed_Limit = 22;
//}
//else if(SHOOT_PID->CAN2_RC_DATA->shooter_id1_17mm_speed_limit == 18)
//{
//	MAX_Shoot_Speed_Limit = 25;
//}
//else if(SHOOT_PID->CAN2_RC_DATA->shooter_id1_17mm_speed_limit == 30)
//{
//	MAX_Shoot_Speed_Limit = 43;
//}
//else
//{
//	MAX_Shoot_Speed_Limit = 30;
//}


/****************************����**************************************/
static long compensation_Tick = 0;
//���Ե���ģʽ
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

////��������ģʽ
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

//��־λ��λ
if(__fabs(SHOOT_PID->Get_Rc->rc.ch[4]) < BULLET_DEADLINE1)
{
		De_Bug_Flag = 0;
	  De_Bug_OnceEnd_Flag = 0;
}
//����

//ң����
//if(SHOOT_PID->Shoot_Mode.Use_Mode == Handle)
//{
//   if(SHOOT_PID->Shoot_Mode.Shoot_Time == Tri_ON && SHOOT_PID->Shoot_Mode.Shoot_Protect!=STOPPAGE)  //���㲦�̲�һ�ε�ʱ��
//	{
// first_time = xTaskGetTickCount();
// second_time = first_time;
// SHOOT_PID->Shoot_Mode.Shoot_Time = Tri_OFF;
//	}
//   if(SHOOT_PID->Shoot_Mode.Shoot_Time == Tri_OFF)
//{
//	first_time = xTaskGetTickCount();
//	if(SHOOT_PID->Shoot_Mode.Shoot_mode == SHOOT_KEEP )
// {
//	 keep_time_OFF = xTaskGetTickCount();  //���ϲ��˳���ʱ��
//	 if(keep_time_OFF - keep_time_ON > KEEP_SHOOT_TIME1) SHOOT_PID->Shoot_Mode.KEEP_MODE = KEEP_YES;
// }
//   if(SHOOT_PID->Shoot_Mode.KEEP_MODE == KEEP_YES)
// {
////	 if(first_time - second_time > KEEP_TRI_TIME1 && 
////		 (SHOOT_PID->CAN2_RC_DATA->shooter_id1_17mm_cooling_limit - SHOOT_PID->CAN2_RC_DATA->shooter_id1_17mm_cooling_heat > 10)
////	    && SHOOT_PID->CAN2_RC_DATA->Refree_ON == 1 && SHOOT_PID->Shoot_Mode.Shoot_Protect!=STOPPAGE) //�в���ϵͳ
////	 {
////		SHOOT_PID->Shoot_Mode.Shoot_Time = Tri_ON;
////	 	SHOOT_PID->Shoot_PID.Shoot_ANGLE_PID.ref += ONECE_TRI_ANGLE;
////	 }
////	 if(first_time - second_time > KEEP_TRI_TIME1 && SHOOT_PID->Shoot_Mode.Shoot_Protect!=STOPPAGE) //�޲���ϵͳ
////	 {
////		SHOOT_PID->Shoot_Mode.Shoot_Time = Tri_ON;
////	 	SHOOT_PID->Shoot_PID.Shoot_ANGLE_PID.ref += ONECE_TRI_ANGLE;		 
////	 }
// }
//}
// if(SHOOT_PID->Shoot_Mode.Shoot_mode == SHOOT_ONECE)
// {
//	SHOOT_PID->Shoot_Mode.KEEP_MODE = KEEP_NO;
//	keep_time_ON = xTaskGetTickCount();
//	SHOOT_PID->Shoot_Mode.Shoot_Time = Tri_ON;
// }
//}
//���or�����Զ�����
//if(SHOOT_PID->gk_DATA->Fire_Siwtch >= 100 && SHOOT_PID->Get_Gimbal_Data->Use_Flag.gimbal_state == Gimbal_Buff)
//{
//	SHOOT_PID->Shoot_Mode.KEEP_MODE = KEEP_YES;
//}
//else if(SHOOT_PID->gk_DATA->Fire_Siwtch <= 100 && SHOOT_PID->Get_Gimbal_Data->Use_Flag.gimbal_state == Gimbal_Buff)
//{
//	SHOOT_PID->Shoot_Mode.KEEP_MODE = KEEP_NO;
//}
 /**********************************PID����**************************************/

	Angle_Pid_Cala(&SHOOT_PID->Shoot_PID.Shoot_ANGLE_PID,SHOOT_PID->Shoot_PID.Shoot_ANGLE_PID.fdb,SHOOT_PID->Shoot_PID.Shoot_ANGLE_PID.ref);
//  if(SHOOT_PID->Shoot_Mode.Shoot_Protect == NOMAL)
// {
//	stoppage_Time = xTaskGetTickCount();
//	reverse_Time = xTaskGetTickCount();
// }
//��ת������һ��
//  if(SHOOT_PID->Shoot_PID.Shoot_ANGLE_PID.err >= 1000)
//	{
//	if((xTaskGetTickCount() - stoppage_Time) > 500)
//		{
//		SHOOT_PID->Shoot_Mode.Shoot_Protect = STOPPAGE;
//		}
//	}	
//	else if(SHOOT_PID->Shoot_Mode.Shoot_Protect == NOMAL)
//		{
//	   stoppage_Time = xTaskGetTickCount();
//	   reverse_Time = xTaskGetTickCount();
//	  }
////��ת�����ڶ���
//  if(SHOOT_PID->Shoot_PID.Shoot_ANGLE_PID.err > ONECE_TRI_ANGLE*3)
//	{
//	  SHOOT_PID->Shoot_Mode.Shoot_Protect = STOPPAGE;
//	}
////��ת������ʩ
//	if(SHOOT_PID->Shoot_Mode.Shoot_Protect == STOPPAGE)
//	{
////		SHOOT_PID->Shoot_PID.Shoot_ANGLE_PID.ref -=ONECE_TRI_ANGLE*0.5f;
////		if(xTaskGetTickCount() - reverse_Time)
////		{
////		 SHOOT_PID->Shoot_Mode.Shoot_Protect = NOMAL;
////		}
//	}
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
	//���PID��������ֹ��һ�εĲ���Ӱ����һ��
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
	
	static float Error_Kaluli;
	static float E_Shoot_Rate;
	 
//	Error_Kaluli= SHOOT_heat_limt->CAN2_RC_DATA->shooter_id1_17mm_cooling_limit - (SHOOT_heat_limt->CAN2_RC_DATA->shooter_id1_17mm_cooling_heat) ;    //��������-��ǰ����==ʣ������
//	
//	 
//   if(Error_Kaluli <_17mm_Heat_of_Projectile*1)    // ʣ������С��һ�ŵ��������
//	 {
//		E_Shoot_Rate = 0.0001;          //����Ƶ��Ϊ��0 
//	 }
//	 else if(Error_Kaluli>=_17mm_Heat_of_Projectile*1&&Error_Kaluli<=_17mm_Heat_of_Projectile*2)  //ʣ�������պõ���2�ŵ��������
//	 {
//		E_Shoot_Rate = SHOOT_heat_limt->CAN2_RC_DATA->shooter_id1_17mm_cooling_rate /_17mm_Heat_of_Projectile;    //������Ƶ��ǹ�����������ȶ�
//	 }
//	 else if (Error_Kaluli <=_17mm_Heat_of_Projectile * 10&&Error_Kaluli >_17mm_Heat_of_Projectile*2)  //ʣ������С�����ҷ���ģʽ����ֵ
//	 {
//		E_Shoot_Rate= (Error_Kaluli/(_17mm_Heat_of_Projectile  * 10)) * (15-(SHOOT_heat_limt->CAN2_RC_DATA->shooter_id1_17mm_cooling_rate /_17mm_Heat_of_Projectile))
//		               + SHOOT_heat_limt->CAN2_RC_DATA->shooter_id1_17mm_cooling_rate /_17mm_Heat_of_Projectile;  //��Ƶ�����������½�
//	 }
//	 else if (Error_Kaluli>_17mm_Heat_of_Projectile * 10&&Error_Kaluli<=SHOOT_heat_limt->CAN2_RC_DATA->shooter_id1_17mm_cooling_limit  )  //ʣ�������������ҷ���ģʽ����ֵ
//	 {                           
//		 E_Shoot_Rate=15;	 //������ƵΪ���ҷ���ģʽ����Ƶ
//	 }
	 E_Shoot_Rate = 30;
	 return E_Shoot_Rate;           //������Ƶ
 }

