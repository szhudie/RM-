/* USER CODE BEGIN USART2_Init 2 */
#include "bsp_dma.h"
#include "bsp_can.h"
#include "stm32f4xx.h"
#include "main.h"
#include "detect_tesk.h"
#define RECEIVELEN 128
uint8_t rc_RxBuffer[RECEIVELEN];
 RC_Ctrl_t RC_CtrlData ; // 遥控器数据
 uint16_t	temp;//未传数据 用于计算收到多少数据

void RC_Init(uint8_t *rx1_buf,uint16_t dma_buf_num)
{
	SET_BIT(huart2.Instance->CR1, USART_CR1_IDLEIE);//开启串口空闲中断.
	HAL_UART_Receive_DMA(&huart2, (uint8_t *)rx1_buf, dma_buf_num);
}



void remote_control_init(void)
{
    RC_Init(rc_RxBuffer,RECEIVELEN);
}

const RC_Ctrl_t *get_remote_control_point(void)
{
    return &RC_CtrlData;
}

void USART2_IRQHandler(void)
{
	if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE) && 
		__HAL_UART_GET_IT_SOURCE(&huart2, UART_IT_IDLE))
	{
		uint16_t tmp = huart2.Instance->SR;
    tmp = huart2.Instance->DR;
    tmp--;
		__HAL_UART_CLEAR_IDLEFLAG(&huart2);
		CLEAR_BIT(huart2.Instance->SR, USART_SR_IDLE);
		__HAL_DMA_DISABLE(huart2.hdmarx);

		temp = huart2.hdmarx->Instance->NDTR; 
//		if((RECEIVELEN - temp) == 18)
//		{
			Get_Remote_info(&RC_CtrlData ,rc_RxBuffer);
		  DetectHook(5);//记录时间
//		}
		HAL_UART_Receive_DMA(&huart2, (uint8_t *)rc_RxBuffer, RECEIVELEN);
		SET_BIT(huart2.Instance->CR1, USART_CR1_IDLEIE);
		DMA1->HIFCR = DMA_FLAG_DMEIF0_4 | DMA_FLAG_FEIF0_4 | DMA_FLAG_HTIF0_4 | DMA_FLAG_TCIF0_4 | DMA_FLAG_TEIF0_4;
		__HAL_DMA_SET_COUNTER(huart2.hdmarx, RECEIVELEN);
		__HAL_DMA_ENABLE(huart2.hdmarx);
	} 
	
}


void Get_Remote_info(RC_Ctrl_t *rc_ctrl ,volatile const uint8_t *sbus_buf)
{

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

