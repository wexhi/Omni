#ifndef GIMBAL_TASK_H__
#define GIMBAL_TASK_H__

#include "struct_typedef.h"
#include "pid.h"
#include "drv_can.h"
#include "rc_potocal.h"
#include "main.h"
#include "can.h"
#include "gpio.h"

/**
 * @brief 云台Pitch轴电机的初始化
 *
 */
void Gimbal_Init();

void Gimbal_task(void const *pvParameters);

#endif // !
