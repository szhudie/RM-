#include "can.h"
#include "bsp_can.h"
#include "bsp_dma.h"
#include "detect_tesk.h"
uint8_t chassis_can_send_data[8]={0};//��������
uint8_t GIMBAL_can_send_data[8]={0};//��̨����
CAN_TxHeaderTypeDef chassis_tx_message;//����can�ṹ��
CAN_TxHeaderTypeDef GIMBAL_TxMessage;//��̨can�ṹ��
motor_measure_t motor_yaw, motor_pit, motor_chassis[4];
//int motor_flag = 1; �������ɾ��
uint8_t sbus_buf[18];//ң��������
//ͳһ����can���պ���
static void CAN_hook(CAN_RxHeaderTypeDef *rx_message,uint8_t rx_data[8]);
void can2_filter_init(void);

void Can_init(void){

can_filter_init();//----��������ʼ��
HAL_CAN_Start(&hcan1);
HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

}
void Can2_init(void){
can2_filter_init();//----��������ʼ��
HAL_CAN_Start(&hcan2);
HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void can_filter_init(void)//can�Ĺ�������ʼ������
{

    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
	  can_filter_st.SlaveStartFilterBank=14;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);

}

void can2_filter_init(void)//can�Ĺ�������ʼ������
{

    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 14;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
	  can_filter_st.SlaveStartFilterBank=14;
	  HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);

}
//���͵��̿�������
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
	  uint32_t send_mail_box;
    chassis_tx_message.StdId =0x200;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&hcan1, &chassis_tx_message, chassis_can_send_data,&send_mail_box);
}

//������̨�����������revΪ�����ֽ�
void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev)
{
	  uint32_t send_mail_box;
    GIMBAL_TxMessage.StdId = CAN_GIMBAL_ALL_ID;
    GIMBAL_TxMessage.IDE = CAN_ID_STD;
    GIMBAL_TxMessage.RTR = CAN_RTR_DATA;
    GIMBAL_TxMessage.DLC = 0x08;
    GIMBAL_can_send_data[0] = (yaw >> 8);
    GIMBAL_can_send_data[1] = yaw;
    GIMBAL_can_send_data[2] = (pitch >> 8);
    GIMBAL_can_send_data[3] = pitch;
    GIMBAL_can_send_data[4] = (shoot >> 8);
    GIMBAL_can_send_data[5] = shoot;
    GIMBAL_can_send_data[6] = (rev >> 8);
    GIMBAL_can_send_data[7] = rev;
    HAL_CAN_AddTxMessage(&hcan1, &GIMBAL_TxMessage, GIMBAL_can_send_data,&send_mail_box);
}


/*
*ecd-ת�ӻ�е�Ƕ�
*speed_rpm-ת��ת��
*given_current-ʵ��ת�ص���
*temperate-�¶�
*/
void get_motor_measure(motor_measure_t *motor_chassis, uint8_t data[8])                                    \
    {  
//			if(motor_flag==1){
//		motor_flag=0;
//			motor_chassis->ecd_flag = ((int16_t)((data)[0] << 8 | (data)[1]));
//		}                    �������ɾ��                             
        motor_chassis->last_ecd=motor_chassis->ecd;        //PID��ǰֵ
        motor_chassis->ecd = ((int16_t)((data)[0] << 8 | (data)[1]));            
        motor_chassis->speed_rpm = (int16_t)((data)[2] << 8 | (data)[3]);      
        motor_chassis->given_current = (int16_t)((data)[4] << 8 | (data)[5]);  
        motor_chassis->temperate = (data)[6]; 

// ��Ȧ
motor_chassis-> angle_error =  motor_chassis->ecd	- motor_chassis->last_ecd;
			if( motor_chassis->angle_error<-8000){
			(motor_chassis->quan)++;
				}else if(motor_chassis->angle_error>8000){
			(motor_chassis->quan)--;
			}
 motor_chassis->ecd_out = motor_chassis->ecd + (motor_chassis->quan)*8192;
					
    }//��������ݷŽ��ṹ��
		

		
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)//-------�жϻص�����------
{
	if(hcan == &hcan1){
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);//�����жϺ󣬵�һ����Ϣ������rx_data
	  CAN_hook(&rx_header,rx_data);
	}
	if(hcan == &hcan2){
	  CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);//�����жϺ󣬵�һ����Ϣ������rx_data
	  CAN_hook(&rx_header,rx_data);
	
	 }
}



//ͳһ����can�жϺ��������Ҽ�¼�������ݵ�ʱ�䣬��Ϊ�����ж�����
static void CAN_hook(CAN_RxHeaderTypeDef *rx_message,uint8_t rx_data[8])
{

    switch (rx_message->StdId)
    {
    case CAN_3508_M1_ID:
    case CAN_3508_M2_ID:
    case CAN_3508_M3_ID:
    case CAN_3508_M4_ID:
    {
        static uint8_t i = 0;
        //������ID��
        i = rx_message->StdId - CAN_3508_M1_ID;
        //���������ݺ꺯��
        get_motor_measure(&motor_chassis[i], rx_data);
        DetectHook(i+1);//��¼ʱ��
        break;
    }
		//YAW����
		case 100:{
		get_motor_measure(&motor_yaw, rx_data);
		DetectHook(0);//��¼ʱ��
			break;
		}
		//ң����
		case 150:{
			char i;
			for(i=0;i<7;i++){
		sbus_buf[i] = rx_data[i];
		 }
			Get_Remote_info(&RC_CtrlData ,sbus_buf);
		 DetectHook(5);//��¼ʱ��
		 break;
		}
		//������ʱ��ûд
    case 0x300:{
 char i;
	for(i=8;i<15;i++){
		sbus_buf[i] = rx_data[i-8];
		 }
//	Get_Remote_info(&RC_CtrlData ,sbus_buf);
		 break;
		}
    default:
    {
        break;
    }
    }
}

//����yaw���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const motor_measure_t *get_Yaw_Gimbal_Motor_Measure_Point(void)
{
    return &motor_yaw;
}
//����pitch���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const motor_measure_t *get_Pitch_Gimbal_Motor_Measure_Point(void)
{
    return &motor_pit;
}
//���ص��̵��������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const motor_measure_t *get_Chassis_Motor_Measure_Point(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];
}


