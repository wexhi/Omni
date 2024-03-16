// app
#include "robot_def.h"
#include "robot_cmd.h"
// module
#include "remote_control.h"
#include "ins_task.h"
#include "general_def.h"
// bsp
#include "bsp_dwt.h"

static RC_ctrl_t *rc_data; // 遥控器数据,初始化时返回
// 机器人控制量,全局变量
Chassis_Ctrl_Cmd_s chassis_cmd_send; // 底盘控制量
Gimbal_Ctrl_Cmd_s gimbal_cmd_send;   // 云台控制量
Shoot_Ctrl_Cmd_s shoot_cmd_send;     // 发射控制量

void RobotCMDInit()
{
    rc_data = RemoteControlInit(&huart3); // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
}

/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
static void RemoteControlSet()
{
    if (switch_is_down(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[下],底盘跟随云台
    {
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    }
    else if (switch_is_mid(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[中],正常模式/赛场模式
    {
        chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    }
    else if (switch_is_up(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[上],紧急停止
    {
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
    }
    else // 紧急停止
    {
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
    }

    if (switch_is_down(rc_data[TEMP].rc.switch_left)) // 左侧开关状态[下],摩擦轮关闭
    {
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        shoot_cmd_send.shoot_mode = SHOOT_OFF;
        shoot_cmd_send.loader_mode = LOAD_STOP;
    }
    else if (switch_is_mid(rc_data[TEMP].rc.switch_left)) // 左侧开关状态[中],摩擦轮开启,发射关闭
    {
        shoot_cmd_send.friction_mode = FRICTION_ON;
        shoot_cmd_send.shoot_mode = SHOOT_OFF;
        shoot_cmd_send.loader_mode = LOAD_STOP;
    }
    else if (switch_is_up(rc_data[TEMP].rc.switch_left)) // 左侧开关状态[上], 摩擦轮开启,发射
    {
        shoot_cmd_send.friction_mode = FRICTION_ON;
        shoot_cmd_send.shoot_mode = SHOOT_ON;
        shoot_cmd_send.loader_mode = LOAD_MIDLE;
    }
    else // 摩擦轮关闭
    {
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        shoot_cmd_send.shoot_mode = SHOOT_OFF;
        shoot_cmd_send.loader_mode = LOAD_STOP;
    }
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask()
{
    RemoteControlSet(); // 遥控器控制量设置
}