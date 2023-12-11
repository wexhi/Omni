#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H
#include "struct_typedef.h"
#include "pid.h"
#include "drv_can.h"
#include "rc_potocal.h"
#include "main.h"
#include "gpio.h"
typedef enum
{
    CHAS_LF,
    CHAS_RF,
    CHAS_RB,
    CHAS_LB,
} chassis_motor_cnt_t;

extern chassis_t chassis;
extern int16_t Drifting_yaw;
extern uint16_t Down_ins_yaw;

void Chassis_task(void const *pvParameters);

#endif
