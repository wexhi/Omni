#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H
#include "struct_typedef.h"
#include "pid.h"
#include "drv_can.h"
#include "main.h"
#include "gpio.h"
typedef enum
{
    CHAS_LF,
    CHAS_RF,
    CHAS_RB,
    CHAS_LB,
} chassis_motor_cnt_t;

typedef struct
{
    /* data */
    motor_info_t motor_info[4]; // 电机信息结构体
    fp32 pid_parameter[3];      // 底盘电机的pid参数
    pid_struct_t pid[4];        // 底盘电机的pid结构体
    int16_t speed_target[4];    // 底盘电机的目标速度
    int16_t Vx, Vy, Wz;         // 底盘电机的目标速度
    fp32 err_angle;             // 下板与上板的角度差
    fp32 err_angle_rad;         // 下板与上板的角度差(弧度制)
    fp32 imu_err;               // 修正陀螺仪漂移量
} chassis_t;

void Chassis_Init(void);
void Chassis_task(void const *pvParameters);

#endif
