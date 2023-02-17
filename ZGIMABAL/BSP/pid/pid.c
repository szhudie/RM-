/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       pid.c/h
  * @brief      pid实现函数，包括初始化，PID计算函数，
  */

#include "pid.h"
#include "bsp_can.h"
#include <string.h>     
#include <stdio.h>
pid_type_def chassis_speed_pid[4];//定义底盘结构体
pid_type_def chassis_angle_pid[4];//定义底盘结构体
const motor_measure_t *motor_data;	//声明电机结构体指针(用于存储can读数)
const pid_type_def *pid_speed;
const pid_type_def *pid_angld;
extern int s0_flag;

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

/*********************************************Fuzzy_PID***************************************************************/
//误差的阀值，小于这个数值的时候，不做PID调整，避免误差较小时频繁调节引起震荡
#define Emin 0.0
#define Emid 0.08
#define Emax 0
//调整值限幅，防止积分饱和
#define Umax 5
#define Umin -5

//输出值限幅
#define Pmax 7200
#define Pmin 0

#define NB -120   //-6
#define NM -80    //-4
#define NS -40    //-1
#define ZO 0      //0
#define PS 40     //1
#define PM 80     //4
#define PB 120    //6

int kp[7][7]={	{PB,PB,PM,PM,PS,ZO,ZO},
				        {PB,PB,PM,PS,PS,ZO,ZO},
				        {PM,PM,PM,PS,ZO,NS,NS},
				        {PM,PM,PS,ZO,NS,NM,NM},
			        	{PS,PS,ZO,NS,NS,NM,NM},
				        {PS,ZO,NS,NM,NM,NM,NB},
			        	{ZO,ZO,NM,NM,NM,NB,NB}    };        

int kd[7][7]={	{PS,NS,NB,NB,NB,NM,PS},
			        	{PS,NS,NB,NM,NM,NS,ZO},
			        	{ZO,NS,NM,NM,NS,NS,ZO},
			        	{ZO,NS,NS,NS,NS,NS,ZO},
			        	{ZO,ZO,ZO,ZO,ZO,ZO,ZO},
			        	{PB,NS,PS,PS,PS,PS,PB},
			        	{PB,PM,PM,PM,PS,PS,PB}    };

int ki[7][7]={	{NB,NB,NM,NM,NS,ZO,ZO},
			        	{NB,NB,NM,NS,NS,ZO,ZO},
			        	{NB,NM,NS,NS,ZO,PS,PS},
			        	{NM,NM,NS,ZO,PS,PM,PM},
			        	{NM,NS,ZO,PS,PS,PM,PB},
			        	{ZO,ZO,PS,PS,PM,PB,PB},
			        	{ZO,ZO,PS,PM,PM,PB,PB}    };

/**************求隶属度（正三角形）***************/
float FTri(float x,float a,float b,float c)//FuzzyTriangle
{
	if(x<=a)
		return 0;
	else if((a<x)&&(x<=b))
		return (x-a)/(b-a);
	else if((b<x)&&(x<=c))
		return (c-x)/(c-b);
	else if  (x>c)
		return 0;
	else
		return 0;
}
/**************求隶属度（倒三角形）***************/
float F_DTri(float x,float a,float b,float c)//FuzzyTriangle
{
	if(x<=a)
		return 1;
	else if((a<x)&&(x<=b))
		return -(x-a)/(b-a)+1;
	else if((b<x)&&(x<=c))
		return -(c-x)/(c-b)+1;
	else if  (x>c)
		return 1;
	else
		return 1;
}
/*****************求隶属度（梯形左）*******************/
float FTraL(float x,float a,float b)//FuzzyTrapezoidLeft
{
	if(x<=a)  
		return 1;
	else if((a<x)&&(x<=b))
		return (b-x)/(b-a);
	else if(x>b)
		return 0;
	else
		return 0;
}
/*****************求隶属度（梯形右）*******************/
float FTraR(float x,float a,float b)//FuzzyTrapezoidRight
{
	if(x<=a)
		return 0;
	if((a<x)&&(x<b))
		return (x-a)/(b-a);
	if(x>=b)
		return 1;
	else
		return 1;
}
/****************三角形反模糊化处理**********************/
float uFTri(float x,float a,float b,float c)
{ 
	float y,z;
	z=(b-a)*x+a;
	y=c-(c-b)*x;
	return (y+z)/2;
}
/*******************梯形（左）反模糊化***********************/
float uFTraL(float x,float a,float b)
{
	return b-(b-a)*x;
}
/*******************梯形（右）反模糊化***********************/
float uFTraR(float x,float a,float b)
{
	return (b-a)*x +a;
}
/**************************求交集****************************/
float fand(float a,float b)
{
	return (a<b)?a:b;
}
/**************************求并集****************************/
float forr(float a,float b)
{
	return (a<b)?b:a;
}


 void Fuzzy_Pid_Init(FUZZY_PID *pid,  const fp32 PID[3],const fp32 fabs_E[4],const fp32 fabs_EC[4], fp32 max_out, fp32 max_iout,int mode,fp32 Fuzzy_decrease_Ratio[3])
{
    if (pid == NULL)
    {
       return;
    }
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
		
	 //误差范围赋值
  	memcpy(pid->fabs_E  ,fabs_E, 16 );
  	memcpy(pid->fabs_EC ,fabs_EC,16 );

		
		pid->mode=mode; // 运行模式：  Calculus_MODE：微分项直接用，calculus输入微分项；NORMAL：正常的误差率算微分,calculus输入0;
		
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
		
    pid->set = 0.0f;
    pid->fdb = 0.0f;

	  pid->Fuzzy_kp_Ratio =Fuzzy_decrease_Ratio[0];//decrease
		pid->Fuzzy_ki_Ratio =Fuzzy_decrease_Ratio[1];
		pid->Fuzzy_kd_Ratio =Fuzzy_decrease_Ratio[2];
		
    pid->max_iout = max_iout;
    pid->max_out = max_out;
		    
}
 float detkp,detki,detkd;
/*======================   PID计算部分   ======================*/   
void Fuzzy_Pid_calc(FUZZY_PID *structpid,float set,float fdb,float calculus)
{
	//计算隶属度表
	  
	int i=0,j=0;
	
	//记录隶属度最大项及相应推理表的p、i、d值
	float lsd;
	int temp_p,temp_d,temp_i;
	
	structpid->fdb =fdb;
	structpid->set =set; 
	
	structpid->erro = theta_format( structpid->set - structpid->fdb);// 偏差(限幅为+-180)
	
	structpid->e  = structpid->erro ;
	structpid->ec = (structpid->erro - structpid->LastError)*50;
	
	//当误差的绝对值小于Emax时，对pid的参数进行调整
	if(__fabs(structpid->e )>=Emax)
	{
		
	detkp=0;
	detki=0;
	detkd=0;
 //计算隶属度表
	structpid->es[0]=FTraL(structpid->e,-structpid->fabs_E[3],-structpid->fabs_E[1]);         //求误差e的隶属度 
	structpid->es[1]=FTri (structpid->e,-structpid->fabs_E[3],-structpid->fabs_E[2],structpid->fabs_E[0]);
	structpid->es[2]=FTri (structpid->e,-structpid->fabs_E[3],-structpid->fabs_E[1],structpid->fabs_E[1]);
	structpid->es[3]=FTri (structpid->e,-structpid->fabs_E[2], structpid->fabs_E[0],structpid->fabs_E[2]);
	structpid->es[4]=FTri (structpid->e,-structpid->fabs_E[1], structpid->fabs_E[1],structpid->fabs_E[3]);
	structpid->es[5]=FTri (structpid->e, structpid->fabs_E[0], structpid->fabs_E[3],structpid->fabs_E[3]);
	structpid->es[6]=FTraR(structpid->e, structpid->fabs_E[1], structpid->fabs_E[3]);

	structpid->ecs[0]=FTraL(structpid->ec,-structpid->fabs_EC[3],-structpid->fabs_EC[1]);      //求误差变化率ec的隶属度
	structpid->ecs[1]=FTri (structpid->ec,-structpid->fabs_EC[3],-structpid->fabs_EC[2],structpid->fabs_EC[0]);   
	structpid->ecs[2]=FTri (structpid->ec,-structpid->fabs_EC[3],-structpid->fabs_EC[1],structpid->fabs_EC[1]);
	structpid->ecs[3]=FTri (structpid->ec,-structpid->fabs_EC[2], structpid->fabs_EC[0],structpid->fabs_EC[2]);
	structpid->ecs[4]=FTri (structpid->ec,-structpid->fabs_EC[1], structpid->fabs_EC[1],structpid->fabs_EC[3]);
	structpid->ecs[5]=FTri (structpid->ec, structpid->fabs_EC[0], structpid->fabs_EC[2],structpid->fabs_EC[3]);
	structpid->ecs[6]=FTraR(structpid->ec, structpid->fabs_EC[1], structpid->fabs_EC[3]);
	//计算隶属度表，确定e和ec相关联后表格各项隶属度的值
	
	
	for(j=0;j<7;j++)             //用重心法求▲kp；
	{
	   for(i=0;i<7;i++)
	   {
	    detkp=detkp+structpid->es[i]*structpid->ecs[j]*kp[i][j];
	   }
  } 
	
 for(j=0;j<7;j++)             //用重心法求▲ki；
	{
	   for(i=0;i<7;i++)
	   {
	    detki=detki+structpid->es[i]*structpid->ecs[j]*ki[i][j];
	   }
  }
	
	for(j=0;j<7;j++)             //用重心法求▲kd；
	{
	   for(i=0;i<7;i++)
	   {
	    detkd=detkd+structpid->es[i]*structpid->ecs[j]*kd[i][j];
	   }
  }

  structpid->LastError=structpid->erro;	
 }
	 structpid->detkp = -detkp ;
   structpid->detki = detki ;
   structpid->detkd = detkd ;
 
	 structpid->fuzzy_kp= structpid->Kp + structpid->detkp/structpid->Fuzzy_kp_Ratio ;
   structpid->fuzzy_ki= structpid->Ki + structpid->detki/structpid->Fuzzy_ki_Ratio  ;
   structpid->fuzzy_kd= structpid->Kd + structpid->detkd/structpid->Fuzzy_kd_Ratio ;
 
	structpid->e = theta_format(structpid->set - structpid->fdb);

	
		  if(structpid->mode==Calculus_MODE)               //微分项直接乘模式
	   {	
	    structpid->Dout=structpid->fuzzy_kd*calculus;
	   }
	   else if (structpid->mode==POSITION)             //位置式pid 模式
	   {
	if(__fabs(  structpid->e ) > 1)
		{
			structpid->Iout= 0;
		}
		else 
		{
		structpid->Iout+=structpid->e*structpid->Ki ;  
		} 
	    structpid->Dbuf[2] = structpid->Dbuf[1];
      structpid->Dbuf[1] = structpid->Dbuf[0];
      structpid->Dbuf[0] = structpid->ec;
	    structpid->Dout= structpid->fuzzy_kd * structpid->Dbuf[0];
	   }
		 else if (structpid ->mode == YAW_POSITION){
		 	if(__fabs(  structpid->e ) > 0.25)
		{
			structpid->Iout= 0;
		}
		else 
		{
		structpid->Iout+=structpid->e*structpid->Ki ;  
		} 
	    structpid->Dbuf[2] = structpid->Dbuf[1];
      structpid->Dbuf[1] = structpid->Dbuf[0];
      structpid->Dbuf[0] = structpid->ec;
	    structpid->Dout= structpid->fuzzy_kd * structpid->Dbuf[0];
		 }
		 
		 structpid->Pout = structpid->fuzzy_kp * structpid->e;
	structpid->out= (structpid->Pout) + (structpid->Iout)+(structpid->Dout) ;
	LimitMax(structpid->out,structpid->max_out);
 
}



