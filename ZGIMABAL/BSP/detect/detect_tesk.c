#include "detect_tesk.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#define errorListLength 3
/* 0:Y��������ж�  */
/* 1:P��������ж�  */
/* 2:���ջ������ж�  */
/* 3:�����Ƕ����ж�  */
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
			//��ֹ�ڴ���������ʱ������ע��ʱ�䣬����żȻ�Ե�ǰʱ��С��ע��ʱ�䡣���ָ����������жϳ���
			if(xTaskGetTickCount()<errorList[i].regTime)
			{
				errorList[i].regTime=xTaskGetTickCount()-1;
			}
			errorList[i].Losttime=(int)(xTaskGetTickCount()-errorList[i].regTime);//��ǰʱ���ȥע��ʱ��
			if(errorList[i].Losttime>errorList[i].overtime)//ʱ�����趨��ʱʱ��
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

//�豸�������ݹ��Ӻ���
void DetectHook(uint8_t toe)//���ú���������ע��ʱ��
{
	errorList[toe].regTime= xTaskGetTickCount();
}
void DetectInit(uint16_t i,uint32_t over_time)//over_time��ʱʱ��
{
		errorList[i].regTime  = xTaskGetTickCount();
		errorList[i].overtime = over_time;
}

//�ж��Ƿ���ߣ�����1����
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
