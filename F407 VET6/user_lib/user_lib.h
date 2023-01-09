#ifndef USER_LIB_H
#define USER_LIB_H
#include "main.h"
#include "struct_typedef.h"
typedef struct ramp_t
{
  int32_t count;
  int32_t scale;
  float   out;
  void  (*init)(struct ramp_t *ramp, int32_t scale);
  float (*calc)(struct ramp_t *ramp);
}ramp_t;

#define RAMP_GEN_DAFAULT \
{ \
              .count = 0, \
              .scale = 0, \
              .out = 0, \
              .init = &ramp_init, \
              .calc = &ramp_calc, \
            } \

typedef __packed struct
{
    fp32 input;        //��������
    fp32 out;          //�˲����������
    fp32 num[1];       //�˲�����
    fp32 frame_period; //�˲���ʱ���� ��λ s
} first_order_filter_type_t;
//���ٿ���
extern fp32 invSqrt(fp32 num);

//б��������ʼ��
void  ramp_init(ramp_t *ramp, int32_t scale);
//б����������
float ramp_calc(ramp_t *ramp);
//һ���˲���ʼ��
extern void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 frame_period, const fp32 num[1]);
//һ���˲�����
extern void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, fp32 input);
//��������
extern void abs_limit(fp32 *num, fp32 Limit);
//�жϷ���λ
extern fp32 sign(fp32 value);
//��������
extern fp32 fp32_deadline(fp32 Value, fp32 minValue, fp32 maxValue);
//int26����
extern int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue);
//�޷�����
extern fp32 fp32_constrain(fp32 Value, fp32 minValue, fp32 maxValue);
//�޷�����
extern int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue);
//ѭ���޷�����
extern fp32 loop_fp32_constrain(fp32 Input, fp32 minValue, fp32 maxValue);
//�Ƕ� ���޷� 180 ~ -180
extern fp32 theta_format(fp32 Ang);
//�Ƕȸ�ʽ��Ϊ-PI~PI
extern fp32 WrapToPi(fp32 Ang);
extern void Gyro_Init(uint32_t Delay);
fp32 Get_Sin(uint32_t max_angle,fp32 add_time);//add_time �Ƕȸı�Ŀ�������СԽ��
#define rad_format(Ang) loop_fp32_constrain((Ang), -PI, PI)
#define RAD2DEG(x) ((x)*57.30)  //����ת�Ƕ� //1rad = 180/�� = 57.30��
#define DEG2RAD(x) ((x)*0.01745)  //�Ƕ�ת����//1��=��/180 �� 0.01745 rad
#endif
