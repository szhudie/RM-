#include "can.h"
#include "bsp_can.h"
#include "detect_tesk.h"
uint8_t chassis_can_send_data[8]={0};//��������
uint8_t GIMBAL_can_send_data[8]={0};//��̨����
CAN_TxHeaderTypeDef chassis_tx_message;//����can�ṹ��
CAN_TxHeaderTypeDef GIMBAL_TxMessage;//��̨can�ṹ��
CAN_TxHeaderTypeDef    Tx_Shoot_Message;
motor_measure_t motor_yaw, motor_pit, motor_trigger, motor_chassis[7];
static motor_measure_t Motor_Pluck;
static motor_measure_t Motor_Fri1; 
static motor_measure_t Motor_Fri2;
int motor_flag = 1;
//ͳһ����can���պ���
static void CAN_hook(CAN_RxHeaderTypeDef *rx_message,uint8_t rx_data[8]);
void Can_init(void){
can_filter_init();//----��������ʼ��
HAL_CAN_Start(&hcan1);
HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void Can2_init(void){
HAL_CAN_Start(&hcan2);
HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}
/*********************************************************************/

//ѭ���޷�����
fp32 loop_fp32_constrain(fp32 Input, fp32 minValue, fp32 maxValue)
{
    if (maxValue < minValue)
    {
        return Input;
    }

    if (Input > maxValue)
    {
        fp32 len = maxValue - minValue;
        while (Input > maxValue)
        {
            Input -= len;
        }
    }
    else if (Input < minValue)
    {
        fp32 len = maxValue - minValue;
        while (Input < minValue)
        {
            Input += len;
        }
    }
    return Input;
}

/*********************************************************************/

//�Ƕȸ�ʽ��Ϊ-180~180
fp32 theta_format(fp32 Ang)
{
    return loop_fp32_constrain(Ang, -180.0f, 180.0f);
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
	  can_filter_st.SlaveStartFilterBank=0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
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

//���Ͳ��̵������
//����Ħ���ֵ������
void send_shoot_cur(int16_t Fri1,int16_t Fri2,int16_t Tri1)
{
  uint8_t Fri_Tri_Data[8];
  Tx_Shoot_Message.RTR = CAN_RTR_DATA;
  Tx_Shoot_Message.IDE = CAN_ID_STD;
  Tx_Shoot_Message.StdId = 0x200;
  Tx_Shoot_Message.DLC = 0x08;
	Fri_Tri_Data[0] = Fri1 >> 8;
	Fri_Tri_Data[1] = Fri1;
	Fri_Tri_Data[2] = Fri2 >> 8;
	Fri_Tri_Data[3] = Fri2;
	Fri_Tri_Data[4] = Tri1 >> 8;
	Fri_Tri_Data[5] = Tri1;	
	Fri_Tri_Data[6] = 0;
	Fri_Tri_Data[7] = 0;
	
  HAL_CAN_AddTxMessage(&hcan1,&Tx_Shoot_Message,Fri_Tri_Data,(uint32_t*)CAN_TX_MAILBOX0);

}

//can2 ���ͨѶ����YAW������
void CAN2_motor(uint8_t rx_data[8]){
	  uint32_t send_mail_box1;
    GIMBAL_TxMessage.StdId = 100;
    GIMBAL_TxMessage.IDE = CAN_ID_STD;
    GIMBAL_TxMessage.RTR = CAN_RTR_DATA;
    GIMBAL_TxMessage.DLC = 0x08;
    GIMBAL_can_send_data[0] = rx_data[0];
    GIMBAL_can_send_data[1] = rx_data[1];
    GIMBAL_can_send_data[2] = rx_data[2];
    GIMBAL_can_send_data[3] = rx_data[3];
    GIMBAL_can_send_data[4] = rx_data[4];
    GIMBAL_can_send_data[5] = rx_data[5];
    GIMBAL_can_send_data[6] = rx_data[6];
    GIMBAL_can_send_data[7] = rx_data[7];
HAL_CAN_AddTxMessage(&hcan2, &GIMBAL_TxMessage, GIMBAL_can_send_data,&send_mail_box1);
}
//���ͨѶң����
void CAN2_remote_get1(uint8_t rx_data[8]){
	  uint32_t send_mail_box1;
    GIMBAL_TxMessage.StdId = 150;
    GIMBAL_TxMessage.IDE = CAN_ID_STD;
    GIMBAL_TxMessage.RTR = CAN_RTR_DATA;
    GIMBAL_TxMessage.DLC = 0x08;
    GIMBAL_can_send_data[0] = rx_data[0];
    GIMBAL_can_send_data[1] = rx_data[1];
    GIMBAL_can_send_data[2] = rx_data[2];
    GIMBAL_can_send_data[3] = rx_data[3];
    GIMBAL_can_send_data[4] = rx_data[4];
    GIMBAL_can_send_data[5] = rx_data[5];
    GIMBAL_can_send_data[6] = rx_data[6];
    GIMBAL_can_send_data[7] = rx_data[7];
HAL_CAN_AddTxMessage(&hcan2, &GIMBAL_TxMessage, GIMBAL_can_send_data,&send_mail_box1);
}

void CAN2_remote_get2(uint8_t rx_data[8]){
	  uint32_t send_mail_box1;
    GIMBAL_TxMessage.StdId = 160;
    GIMBAL_TxMessage.IDE = CAN_ID_STD;
    GIMBAL_TxMessage.RTR = CAN_RTR_DATA;
    GIMBAL_TxMessage.DLC = 0x08;
    GIMBAL_can_send_data[0] = rx_data[8];
    GIMBAL_can_send_data[1] = rx_data[9];
    GIMBAL_can_send_data[2] = rx_data[10];
    GIMBAL_can_send_data[3] = rx_data[11];
    GIMBAL_can_send_data[4] = rx_data[12];
    GIMBAL_can_send_data[5] = rx_data[13];
    GIMBAL_can_send_data[6] = rx_data[14];
    GIMBAL_can_send_data[7] = rx_data[15];
HAL_CAN_AddTxMessage(&hcan2, &GIMBAL_TxMessage, GIMBAL_can_send_data,&send_mail_box1);
}
void CAN2_get_remote(uint8_t *rx_data){
CAN2_remote_get1(rx_data);
//CAN2_remote_get2(rx_data);
}


/*
*ecd-ת�ӻ�е�Ƕ�
*speed_rpm-ת��ת��
*given_current-ʵ��ת�ص���
*temperate-�¶�
*/
void get_motor_measure(motor_measure_t *motor_chassis, uint8_t data[8])                                    \
    {  
			if(motor_flag==1){
		motor_flag=0;
			motor_chassis->ecd_flag = ((int16_t)((data)[0] << 8 | (data)[1]));
		}                                                
        motor_chassis->last_ecd=motor_chassis->ecd;        //PID��ǰֵ
        motor_chassis->ecd = ((int16_t)((data)[0] << 8 | (data)[1]));            
        motor_chassis->speed_rpm = (int16_t)((data)[2] << 8 | (data)[3]);      
        motor_chassis->given_current = (int16_t)((data)[4] << 8 | (data)[5]);  
        motor_chassis->temperate = (data)[6]; 


motor_chassis-> angle_error =  motor_chassis->ecd	- motor_chassis->last_ecd;
			if( motor_chassis->angle_error<-1000){
			(motor_chassis->quan)++;
				}else if(motor_chassis->angle_error>1000){
			(motor_chassis->quan)--;
			}
 motor_chassis->ecd_out = motor_chassis->ecd + (motor_chassis->quan)*8192;
					
    }//��������ݷŽ��ṹ��
		
/*********************************************************************/

//���̵�����ݶ�ȡ

void Get_PLUCK_Motor_Data(motor_measure_t* ptr, uint8_t Data[])                                                     
{
			static	int32_t round_cnt,total_ecd,angle_cnt;    
			ptr->last_ecd = ptr->ecd;
			ptr->ecd      = (uint16_t)(Data[0] << 8 | Data[1]);
			(ptr)->speed_rpm = (uint16_t)(Data[2] << 8 | Data[3]);     
			if (ptr->ecd - ptr->last_ecd > 4096)
			{
					round_cnt--;
			}
			else if (ptr->ecd - ptr->last_ecd < -4096)
			{
					round_cnt++;
			}
			total_ecd = round_cnt * 8192 + ptr->ecd;
			/* total angle, unit is degree */
			ptr->total_angle = total_ecd / (8192.0f/360.0f);
			
}
/*********************************************************************/
//Ħ���ֵ�����ݶ�ȡ
void Get_FRI_Motor_Data(motor_measure_t* ptr, uint8_t Data[])                                                     
{   
	static	int32_t total_ecd,round_cnt;
	(ptr)->speed_rpm = (uint16_t)(Data[2] << 8 | Data[3]);                                 
	ptr->last_ecd = ptr->ecd;
	ptr->ecd      = (uint16_t)(Data[0] << 8 | Data[1]);

			if (ptr->ecd - ptr->last_ecd > 4096)
			{
					round_cnt--;
			}
			else if (ptr->ecd - ptr->last_ecd < -4096)
			{
					round_cnt++;
			}
			total_ecd = round_cnt * 8192 + ptr->ecd ;
			/* total angle, unit is degree */
			ptr->total_angle = theta_format(total_ecd / (8192.0f/360.0f));
			
			ptr->angular_velocity = (ptr->speed_rpm*3.14/30.0f)/19.21; //���ٶ� 2*��*N/60/19.21  ��60�� 19.21Ϊת�ٱȣ�
}
		
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)//-------�жϻص�����------
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);//�����жϺ󣬵�һ����Ϣ������rx_data
	  CAN_hook(&rx_header,rx_data);

}

//ͳһ����can�жϺ��������Ҽ�¼�������ݵ�ʱ�䣬��Ϊ�����ж�����
static void CAN_hook(CAN_RxHeaderTypeDef *rx_message,uint8_t rx_data[8])
{
    switch (rx_message->StdId)
    {
    case CAN_YAW_MOTOR_ID:
    {
        //���������ݺ꺯��
			  CAN2_motor(rx_data);
        get_motor_measure(&motor_yaw, rx_data);
        DetectHook(0);
        break;
    }
    case CAN_PIT_MOTOR_ID:
    {
        //���������ݺ꺯��
        get_motor_measure(&motor_pit, rx_data);
        DetectHook(1);
        break;
    }
    case CAN_TRI_MOTOR_ID:
			{
         Get_PLUCK_Motor_Data(&Motor_Pluck, rx_data); 
			}break;
		case CAN_FRI_MOTOR_ID1:
			{
         Get_FRI_Motor_Data(&Motor_Fri1, rx_data); 
			}break;
		case CAN_FRI_MOTOR_ID2:
			{
         Get_FRI_Motor_Data(&Motor_Fri2, rx_data); 
			}break;

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
//����trigger���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const motor_measure_t *get_Trigger_Motor_Measure_Point(void)
{
    return &motor_trigger;
}
//���ص��̵��������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const motor_measure_t *get_Chassis_Motor_Measure_Point(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];
}
//����Ħ���ֵ��������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const motor_measure_t *Get_Fri1_Motor_Data_Point(void)
{
	return &Motor_Fri1;
}
const motor_measure_t *Get_Fri2_Motor_Data_Point(void)
{
	return &Motor_Fri2;
}
//���ز��̵��������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const motor_measure_t *Get_Pluck_Motor_Data_Point(void)
{
	return &Motor_Pluck;
}

