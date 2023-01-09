
#include "Nground.h"
uint8_t nclink_databuf[100];//待发送数据缓冲区
static uint8_t NCLink_Head[2]={0xFF,0xFC};//数据帧头
static uint8_t NCLink_End[2] ={0xA1,0xA2};//数据帧尾

union
{
	unsigned char floatByte[4];
	float floatValue;
}FloatUnion;

void Serial_Data_Send(uint8_t *buf, uint32_t cnt)  
{
HAL_UART_Transmit(&huart6,buf ,cnt,10);
}

void Float2Byte(float *FloatValue, unsigned char *Byte, unsigned char Subscript)
{
	FloatUnion.floatValue = (float)2;
	if(FloatUnion.floatByte[0] == 0)//小端模式
	{
		FloatUnion.floatValue = *FloatValue;
		Byte[Subscript]     = FloatUnion.floatByte[0];
		Byte[Subscript + 1] = FloatUnion.floatByte[1];
		Byte[Subscript + 2] = FloatUnion.floatByte[2];
		Byte[Subscript + 3] = FloatUnion.floatByte[3];
	}
	else//大端模式
	{
		FloatUnion.floatValue = *FloatValue;
		Byte[Subscript]     = FloatUnion.floatByte[3];
		Byte[Subscript + 1] = FloatUnion.floatByte[2];
		Byte[Subscript + 2] = FloatUnion.floatByte[1];
		Byte[Subscript + 3] = FloatUnion.floatByte[0];
	}
}
void NCLink_Send_Userdata(float userdata1	 ,float userdata2,
											    float userdata3  ,float userdata4,
											    float userdata5  ,float userdata6)
{
  uint8_t sum=0,_cnt=0,i=0;
  nclink_databuf[_cnt++]=NCLink_Head[0];
  nclink_databuf[_cnt++]=NCLink_Head[1];
  nclink_databuf[_cnt++]=NCLINK_USER;
  nclink_databuf[_cnt++]=0;
		
	Float2Byte(&userdata1,nclink_databuf,_cnt);//4
	_cnt+=4;
	Float2Byte(&userdata2,nclink_databuf,_cnt);//8
	_cnt+=4;
	Float2Byte(&userdata3,nclink_databuf,_cnt);//12
	_cnt+=4;
	Float2Byte(&userdata4,nclink_databuf,_cnt);//16
	_cnt+=4;
	Float2Byte(&userdata5,nclink_databuf,_cnt);//20
	_cnt+=4;
	Float2Byte(&userdata6,nclink_databuf,_cnt);//24
	_cnt+=4;//28

  nclink_databuf[3] = _cnt-4;
	
  for(i=0;i<_cnt;i++) sum ^= nclink_databuf[i]; 
  nclink_databuf[_cnt++]=sum;
	
	nclink_databuf[_cnt++]=NCLink_End[0];
	nclink_databuf[_cnt++]=NCLink_End[1];
  Serial_Data_Send(nclink_databuf,_cnt);
}

