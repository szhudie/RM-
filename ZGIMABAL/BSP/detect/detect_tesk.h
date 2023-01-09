#ifndef DETECT_Task_H
#define DETECT_Task_H
#include "main.h"
typedef __packed struct
{
    uint32_t regTime;
    int Losttime;
    uint32_t overtime;
		uint8_t  lost_flag;
		uint32_t OFF_TIME;
} error_t;
extern void DetectTask(void *pvParameters);
extern void DetectHook(uint8_t toe);
extern uint8_t Detect_Judeg(uint8_t toe);
extern void DetectInit(uint16_t i,uint32_t over_time);
#endif
