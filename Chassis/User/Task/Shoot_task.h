#ifndef SHOOT_TASK_H__
#define SHOOT_TASK_H__

#include "struct_typedef.h"
#include "pid.h"
#include "drv_can.h"
#include "rc_potocal.h"
#include "main.h"
#include "gpio.h"

void Shoot_task(void const *pvParameters);

#endif // !SHOOT_TASK_H__