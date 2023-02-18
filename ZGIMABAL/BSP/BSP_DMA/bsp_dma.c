/* USER CODE BEGIN USART2_Init 2 */
#include "bsp_dma.h"
#include "detect_tesk.h"
#include "bsp_can.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#define RECEIVELEN 128
#define gk_data_size 18
uint8_t rc_RxBuffer[RECEIVELEN]; //接收串口数据的存储数组
uint8_t gk_RxBuffer[gk_data_size];
PC_GET_DATA pc_data = {0};
 RC_Ctrl_t RC_CtrlData ; //遥控器结构体类型
 uint16_t	temp; //判断接收是否18字节
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;  
//extern uint32_t Reset;//判断遥控是否掉线
void RC_Init(uint8_t *rx1_buf,uint16_t dma_buf_num)
{
	SET_BIT(huart3.Instance->CR1, USART_CR1_IDLEIE);//开启串口空闲中断.
	HAL_UART_Receive_DMA(&huart3, (uint8_t *)rx1_buf, dma_buf_num);
	SET_BIT(huart6.Instance->CR1, USART_CR1_IDLEIE);		//开启“串口总线空闲中断”标志
	HAL_UART_Receive_DMA(&huart6, (uint8_t*)gk_RxBuffer, gk_data_size);//设置要在DMA上接收传输的数据数量
	
}

void remote_control_init(void)
{
    RC_Init(rc_RxBuffer,RECEIVELEN);
}

const RC_Ctrl_t *get_remote_control_point(void)
{
    return &RC_CtrlData;
}

void USART3_IRQHandler(void)
{
	if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE) && 
		__HAL_UART_GET_IT_SOURCE(&huart3, UART_IT_IDLE))
	{
		uint16_t tmp = huart3.Instance->SR;
    tmp = huart3.Instance->DR;
    tmp--;
		__HAL_UART_CLEAR_IDLEFLAG(&huart3);
		CLEAR_BIT(huart3.Instance->SR, USART_SR_IDLE);
		__HAL_DMA_DISABLE(huart3.hdmarx);

		temp = huart3.hdmarx->Instance->NDTR; 
		if((RECEIVELEN - temp) == 18)
		{
			Get_Remote_info(&RC_CtrlData ,rc_RxBuffer);
			DetectHook(2);
		}
		HAL_UART_Receive_DMA(&huart3, (uint8_t *)rc_RxBuffer, RECEIVELEN);

		
		SET_BIT(huart3.Instance->CR1, USART_CR1_IDLEIE);
		DMA1->HIFCR = DMA_FLAG_DMEIF0_4 | DMA_FLAG_FEIF0_4 | DMA_FLAG_HTIF0_4 | DMA_FLAG_TCIF0_4 | DMA_FLAG_TEIF0_4;
		__HAL_DMA_SET_COUNTER(huart3.hdmarx, RECEIVELEN);
		__HAL_DMA_ENABLE(huart3.hdmarx);

	} 
	
}

void USART6_IRQHandler(void)   //工控接收中断
{
	static uint16_t temp;
	
	if (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_IDLE) && 
      __HAL_UART_GET_IT_SOURCE(&huart6, UART_IT_IDLE))  //检测到空闲线路
 {
			uint16_t tmp = huart6.Instance->SR;
      tmp = huart6.Instance->DR;
			//读取DR,SR寄存器中的值否则卡中断
      tmp--;
			__HAL_UART_CLEAR_IDLEFLAG(&huart6); //清除中断标志
	     CLEAR_BIT(huart6.Instance->SR, USART_SR_IDLE);
			__HAL_DMA_DISABLE(huart6.hdmarx); //DMA失能
     	temp = huart6.hdmarx->Instance->NDTR;  //读取DMA数据寄存器剩余量
			
//	if((gk_data_size - temp) == 1)
//	{
		if(gk_RxBuffer[0] == 0xAA && gk_RxBuffer[1] == 0xAA && gk_RxBuffer[13] == 0xBB)    //判断标识位
		{
			
      pc_data.raw_YAW_angle = gk_RxBuffer[2] << 24| gk_RxBuffer[3] << 16 | gk_RxBuffer[4] << 8| gk_RxBuffer[5];
			pc_data.raw_PIT_angle = gk_RxBuffer[6] << 24| gk_RxBuffer[7] << 16 | gk_RxBuffer[8] << 8| gk_RxBuffer[9];
			pc_data.aim_found = gk_RxBuffer[10];
      pc_data.Fire_Siwtch = gk_RxBuffer[11];
			pc_data.Auto_Mode = gk_RxBuffer[12];
			pc_data.PC_Time = gk_RxBuffer[13] << 24| gk_RxBuffer[14] << 16 | gk_RxBuffer[15] << 8| gk_RxBuffer[16];
			pc_data.MiniPC_Rc_Time = xTaskGetTickCount();
		}
//	}
	HAL_UART_Receive_DMA(&huart6, (uint8_t*)gk_RxBuffer, gk_data_size);//设置要在DMA上接收传输的数据数量
	SET_BIT(huart6.Instance->CR1, USART_CR1_IDLEIE);		//开启“串口总线空闲中断”标志
	DMA2->LIFCR = DMA_FLAG_DMEIF1_5 | DMA_FLAG_FEIF1_5 | DMA_FLAG_HTIF1_5 | DMA_FLAG_TCIF1_5 | DMA_FLAG_TEIF1_5;
 
 __HAL_DMA_SET_COUNTER(huart6.hdmarx, gk_data_size);
 __HAL_DMA_ENABLE(huart6.hdmarx); //DMA使能

 }

}

void Get_Remote_info(RC_Ctrl_t *rc_ctrl ,uint8_t *sbus_buf)
{
	  CAN2_get_remote(sbus_buf);//板间通讯
    rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
    rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
                         (sbus_buf[4] << 10)) &0x07ff;
    rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                  //!< Switch left
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;	//!< Switch right
	
	
    rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
    rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis
    rc_ctrl->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press ?
    rc_ctrl->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press ?
    rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value
    rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 //NULL

    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
		
}


uint8_t send_buf[21];
void Ni_Ming(uint8_t fun,float Pid_ref1,float Pid_ref2,float Pid_ref3,float Pid_ref4)
{
HAL_UART_Transmit(&huart6,send_buf,21,10);
}



