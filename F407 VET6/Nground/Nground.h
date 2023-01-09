#ifndef NGROUND_H
#define NGROUND_H

#include "main.h"
#include "usart.h"
#define NCLINK_USER           0x09
extern uint8_t tbuf[16];
void NCLink_Send_Userdata(float userdata1	 ,float userdata2,
											    float userdata3  ,float userdata4,
											    float userdata5  ,float userdata6);

#endif
