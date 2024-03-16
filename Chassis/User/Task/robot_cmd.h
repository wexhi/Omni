#ifndef ROBOT_CMD_H
#define ROBOT_CMD_H
#include "robot_def.h"
#include "struct_typedef.h"


#pragma pack(1) // 1字节对齐
// 底盘模式设置
typedef struct 
{
    // 控制部分
    int16_t vx;           // 前进方向速度
    int16_t vy;           // 横移方向速度
    int16_t wz;           // 旋转速度
    chassis_mode_e chassis_mode;
} Chassis_Ctrl_Cmd_s;

// 云台模式设置
typedef struct 
{
    // 控制部分
    float pitch;        // 俯仰角
    float yaw;          // 偏航角
    gimbal_mode_e gimbal_mode;
} Gimbal_Ctrl_Cmd_s;

// 发射模式设置
typedef struct 
{
    // 控制部分
    shoot_mode_e shoot_mode;
    friction_mode_e friction_mode;
    loader_mode_e loader_mode;
    uint8_t fiction_flag; // 摩擦轮标志位,用于解决键鼠模式下摩擦轮控制
} Shoot_Ctrl_Cmd_s;



/**
 * @brief 机器人核心控制任务初始化,会被RobotInit()调用
 *
 */
void RobotCMDInit();

/**
 * @brief 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率)
 *
 */
void RobotCMDTask();

#endif // !ROBOT_CMD_H