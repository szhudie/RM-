#ifndef IST8310_H
#define IST8310_H
#define READ_IST8310 0X0E<<1|0x01
#define WRITE_IST8310	0X0E<<1
#define IST8310_ADRESS 0X0E

void IST8310_IIC_Init(void);
void IST8310_IIC_WHO_AM_I(void);


#endif
