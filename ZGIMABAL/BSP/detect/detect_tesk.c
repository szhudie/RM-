#include "detect_tesk.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#define errorListLength 3
/* 0:Y电机断线判断  */
/* 1:P电机断线判断  */
/* 2:接收机断线判断  */
/* 3:陀螺仪断线判断  */
static error_t errorList[errorListLength+1];
void DetectTask(void *argument)
{
DetectInit(0,1000);
DetectInit(1,1000);
DetectInit(2,1000);
DetectInit(3,1500);
vTaskDelay(10);
  for(;;)
  {
    		for(uint8_t i=0;i<errorListLength;i++)
		{
			//防止在此任务运行时，正好注册时间，出现偶然性当前时间小于注册时间。出现负数，导致判断出错
			if(xTaskGetTickCount()<errorList[i].regTime)
			{
				errorList[i].regTime=xTaskGetTickCount()-1;
			}
			errorList[i].Losttime=(int)(xTaskGetTickCount()-errorList[i].regTime);//当前时间减去注册时间
			if(errorList[i].Losttime>errorList[i].overtime)//时间差超过设定超时时间
			{
					errorList[i].OFF_TIME++;
					if(errorList[i].OFF_TIME>500)
					{
						errorList[i].OFF_TIME=0;
						errorList[i].lost_flag=1;
					}
			}
			else
			{
				errorList[i].OFF_TIME=0;
				errorList[i].lost_flag=0;
			}
		}
		vTaskDelay(2);
	}  
}

//设备接收数据钩子函数
void DetectHook(uint8_t toe)//调用函数，更行注册时间
{
	errorList[toe].regTime= xTaskGetTickCount();
}
void DetectInit(uint16_t i,uint32_t over_time)//over_time超时时间
{
		errorList[i].regTime  = xTaskGetTickCount();
		errorList[i].overtime = over_time;
}

//判断是否掉线，返回1掉线
uint8_t Detect_Judeg(uint8_t toe)
{
	if(errorList[toe].lost_flag==1)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}
