#ifndef GIMBAL_TASK_H__
#define GIMBAL_TASK_H__

#include "struct_typedef.h"
#include "pid.h"
#include "drv_can.h"
#include "rc_potocal.h"
#include "main.h"
#include "gpio.h"


// typedef struct
// {
//     motor_info_t motor_info;     // 电机信息结构体
//     fp32 pid_parameter[3];       // 云台电机的pid参数
//     fp32 pid_angle_parameter[3]; // 云台电机的pid参数
//     pid_struct_t pid;            // 云台电机的pid结构体
//     pid_struct_t pid_angle;      // 云台电机的pid结构体
//     fp32 speed_target;           // 云台电机的目标速度
//     fp32 angle_target;           // 云台电机的目标角度
//     fp32 init_angle;             // 云台电机的初始角度
// } gimbal_t;

void Gimbal_task(void const *pvParameters);

#endif // !
