#include "Gimbal_task.h"
#include "cmsis_os.h"
#include "INS_task.h"
#include "exchange.h"
#include "drv_can.h"
#define MAX_SPEED 200 // YAW云台电机在速度模式下最大速度

gimbal_t gimbal_Yaw; // 云台电机信息结构体
float yaw_aim;       // 云台YAW轴目标，用于自瞄

extern INS_t INS;               // IMU信息结构体
extern uint8_t is_track;        // 是否自瞄
extern UP_C_angle_t UP_C_angle; // 上C的陀螺仪数据
extern RC_ctrl_t rc_ctrl;       // 遥控器信息结构体

static void mode_select();           // 模式选择
static void gimbal_current_give();   // 云台电机的任务
static void RC_Stop();               // 遥控器控制云台电机
static void RC_Yaw_control();        // 锁云台模式
static void detel_calc(fp32 *angle); // 云台角度计算，防止角度突变

void Gimbal_task(void const *pvParameters)
{
    for (;;)
    {
        mode_select();         // 模式选择
        gimbal_current_give(); // 云台电机的任务
        osDelay(1);
    }
}

/**
 * @brief YAW云台任务初始化
 *
 */
void Gimbal_Init()
{
    // 初始化pid参数
    gimbal_Yaw.pid_parameter[0] = 80, gimbal_Yaw.pid_parameter[1] = 0.5, gimbal_Yaw.pid_parameter[2] = 0;
    gimbal_Yaw.pid_angle_parameter[0] = 9, gimbal_Yaw.pid_angle_parameter[1] = 0, gimbal_Yaw.pid_angle_parameter[2] = 200;
    gimbal_Yaw.angle_target = 0;

    // 初始化pid结构体
    pid_init(&gimbal_Yaw.pid, gimbal_Yaw.pid_parameter, 20000, 10000);
    pid_init(&gimbal_Yaw.pid_angle, gimbal_Yaw.pid_angle_parameter, 20000, 10000);
}

/**
 * @brief 模式选择，现有两种模式，速度模式和锁云台模式
 *
 */
static void mode_select()
{
    if (rc_ctrl.rc.s[0] == 1)
        RC_Stop(); // 停止模式
    else
        RC_Yaw_control(); // 锁云台模式
}

/**
 * @brief 云台电机PID计算并使能电机
 *
 */
static void gimbal_current_give()
{
    gimbal_Yaw.motor_info.set_current = pid_calc(&gimbal_Yaw.pid, -57.3f * UP_C_angle.yaw_gyro, gimbal_Yaw.speed_target);
    if (rc_ctrl.rc.s[0] == 1)
        set_motor_current_gimbal(0, 0, 0, 0, 0);
    else
        set_motor_current_gimbal(0, 0, 0, gimbal_Yaw.motor_info.set_current, 0);
}

/**
 * @brief 停止模式，更新角度目标
 *
 */
static void RC_Stop()
{
    gimbal_Yaw.angle_target = UP_C_angle.yaw; // 保证切换模式后角度不突变
    set_motor_current_gimbal(0, 0, 0, 0, 0);
}

/**
 * @brief 锁云台模式
 *
 * @todo 可以修改小陀螺模式时鼠标控制的灵敏度，等连上图传可以试试
 */
static void RC_Yaw_control()
{
    if (rc_ctrl.rc.ch[0] >= -660 && rc_ctrl.rc.ch[0] <= 660) // 遥控器YAW轴控制
    {
        if (is_track) // if (rc_ctrl.mouse.press_r && is_track)鼠标右键按下且识别到敌方装甲时，云台YAW轴目标为自瞄目标
            gimbal_Yaw.angle_target = yaw_aim;
        else
            gimbal_Yaw.angle_target += rc_ctrl.rc.ch[0] / 660.0 * (-0.6) - // 遥控器控制
                                       (rc_ctrl.mouse.x / 16384.00 * 50);  // 鼠标控制

        detel_calc(&gimbal_Yaw.angle_target);                                                                          // 防止角度突变
        gimbal_Yaw.err_angle = gimbal_Yaw.angle_target - UP_C_angle.yaw;                                               // 计算角度误差
        detel_calc(&gimbal_Yaw.err_angle);                                                                             // 防止角度突变                                                       // 判断是否需要PID控制
        gimbal_Yaw.speed_target = gimbal_Yaw_PID_calc(&gimbal_Yaw.pid_angle, UP_C_angle.yaw, gimbal_Yaw.angle_target); // PID控制
    }
}

/**
 * @brief 云台角度计算，防止角度突变
 *
 * @param angle 需要防止突变的角度
 */
static void detel_calc(fp32 *angle)
{
    if (*angle > 360)
        *angle -= 360;
    else if (*angle < 0)
        *angle += 360;
}
