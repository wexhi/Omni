#ifndef SHOOT_TASK_H__
#define SHOOT_TASK_H__

#include "struct_typedef.h"
#include "pid.h"
#include "drv_can.h"
#include "rc_potocal.h"
#include "main.h"
#include "gpio.h"

typedef struct
{
    /* data */
    motor_info_t motor_info[4]; // 电机信息结构体

    fp32 pid_dial_para[3];     // 拨盘电机的pid参数
    fp32 pid_friction_para[3]; // 摩擦轮电机的pid参数
    fp32 pid_bay_para[3];      // 弹舱电机的pid参数

    pid_struct_t pid_dial;     // 拨盘电机的pid结构体
    pid_struct_t pid_friction; // 摩擦轮电机的pid结构体
    pid_struct_t pid_bay;      // 弹舱电机的pid结构体

    int16_t dial_speed_target;        // 拨盘电机的目标速度
    int16_t friction_speed_target[2]; // 摩擦轮电机的目标速度
    int16_t bay_speed_target;         // 弹舱电机的目标速度

    uint16_t shoot_heat;       // 发射机构的热量
    uint16_t shoot_heat_limit; // 发射机构的热量限制
    uint16_t cooling_value;    // 发射机构的冷却值
    uint8_t shoot_type;        // 发射模式
} shooter_t;

void Shoot_task(void const *pvParameters);

void Shooter_Inint(void);

#endif // !SHOOT_TASK_H__