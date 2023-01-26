#include "user_lib.h"
#include "arm_math.h"
#include "stm32f4xx_hal.h"
#define int_abs(x) ((x) > 0 ? (x) : (-x))
#define Ang_2PI 2*PI
ramp_t chassis_fb_ramp;
//快速开方
fp32 invSqrt(fp32 num)
{
    fp32 halfnum = 0.5f * num;
    fp32 y = num;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(fp32 *)&i;
    y = y * (1.5f - (halfnum * y * y));
    return y;
}

void ramp_init(ramp_t *ramp, int32_t scale)
{
  ramp->count = 0;
  ramp->scale = scale;
}
float ramp_calc(ramp_t *ramp)
{
  if (ramp->scale <= 0)
	{
		return 0;
  }
	
  if (ramp->count++ >= ramp->scale)
	{
		ramp->count = ramp->scale;
	}
	if(ramp==&chassis_fb_ramp)
	{
		if(ramp->count<0.5f*ramp->scale)
		{
			ramp->count=ramp->scale*0.5f;
		}
	}
  ramp->out = ramp->count / ((float)ramp->scale);
  return ramp->out;
}
/**
  * @brief          一阶低通滤波初始化
  * @author         RM
  * @param[in]      一阶低通滤波结构体
  * @param[in]      间隔的时间，单位 s
  * @param[in]      滤波参数
  * @retval         返回空
  */
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 frame_period, const fp32 num[1])
{
    first_order_filter_type->frame_period = frame_period;
    first_order_filter_type->num[0] = num[0];
    first_order_filter_type->input = 0.0f;
    first_order_filter_type->out = 0.0f;
}

/**
  * @brief          一阶低通滤波计算
  * @author         RM
  * @param[in]      一阶低通滤波结构体
  * @param[in]      间隔的时间，单位 s
  * @retval         返回空
  */
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, fp32 input)
{
    first_order_filter_type->input = input;
    first_order_filter_type->out =
        first_order_filter_type->num[0] / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->out + first_order_filter_type->frame_period / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->input;
}

//绝对限制
void abs_limit(fp32 *num, fp32 Limit)
{
    if (*num > Limit)
    {
        *num = Limit;
    }
    else if (*num < -Limit)
    {
        *num = -Limit;
    }
}

//判断符号位
fp32 sign(fp32 value)
{
    if (value >= 0.0f)
    {
        return 1.0f;
    }
    else
    {
        return -1.0f;
    }
}

//浮点死区
fp32 fp32_deadline(fp32 Value, fp32 minValue, fp32 maxValue)
{
    if (Value < maxValue && Value > minValue)
    {
        Value = 0.0f;
    }
    return Value;
}

//int26死区
int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue)
{
    if (Value < maxValue && Value > minValue)
    {
        Value = 0;
    }
    return Value;
}

//限幅函数
fp32 fp32_constrain(fp32 Value, fp32 minValue, fp32 maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}
//限幅函数
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}
//循环限幅函数
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

//角度格式化为-180~180
fp32 theta_format(fp32 Ang)
{
    return loop_fp32_constrain(Ang, -180.0f, 180.0f);
}

//弧度格式化为-PI~PI
fp32 WrapToPi(fp32 Ang)
{
	Ang = Ang+PI;
	Ang = Ang-Ang_2PI*(int)(Ang/(Ang_2PI));
	if(Ang<0)
	{
		Ang+=Ang_2PI;
	}
	return Ang-PI;
}

void Gyro_Init(uint32_t Delay)
{
  uint32_t tickstart = HAL_GetTick();
  uint32_t wait = Delay;

  /* Add a freq to guarantee minimum wait */
  if (wait < HAL_MAX_DELAY)
  {
    wait += (uint32_t)(1U);
  }

  while((HAL_GetTick() - tickstart) < wait)
  {
//		send_6623_jiaozhun();
//		send_Gyro_jiaozhun(0x30,Delay);
  }
}
fp32 Get_Sin(uint32_t max_angle,fp32 add_time)//add_time 角度改变的快慢，最小越快
{
		static fp32 time  = 0.0f;
		static fp32 angle = 0.0f;
		//add_time_copy 角度改变的快慢，最大越快
		fp32  add_time_copy  = PI / add_time;

		angle = max_angle * arm_sin_f32(time);
		time += add_time_copy;
		//函数不超过2pi
		if (time > 2 * PI)
		{
				time -= 2 * PI;
		}
		return angle;
}
