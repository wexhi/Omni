#include "Gimbal_task.h"
#include "cmsis_os.h"
#include "INS_task.h"
#include "exchange.h"
#include "drv_can.h"
#include "miniPC_process.h"
#include "stm32f4xx_it.h"
#define MAX_SPEED 200
#define MAX_ANGLE 200
#define MIN_ANGLE 160

gimbal_t gimbal_Pitch;             // 云台电机信息结构体
static attitude_t *gimba_IMU_data; // 云台IMU数据指针
extern RC_ctrl_t rc_ctrl;          // 遥控器信息结构体
extern Vision_Recv_s *recv;        // 视觉接收信息结构体

static void Pitch_control();          // 遥控器控制云台电机
static void gimbal_current_give();    // 云台电机的任务
static void Angle_Limit(fp32 *angle); // 限制角度
static void detel_calc2(fp32 *angle); // 角度差计算

void Gimbal_task(void const *pvParameters)
{
    for (;;)
    {
        Pitch_control();
        gimbal_current_give();
        osDelay(1);
    }
}

/**
 * @brief 云台Pitch轴电机的初始化
 *
 */
void Gimbal_Init()
{
    gimba_IMU_data = INS_Init(); // IMU先初始化,获取姿态数据指针赋给yaw电机的其他数据来源

    // 初始化pid参数
    gimbal_Pitch.pid_parameter[0] = 80, gimbal_Pitch.pid_parameter[1] = 0, gimbal_Pitch.pid_parameter[2] = 0;
    gimbal_Pitch.pid_angle_parameter[0] = 10, gimbal_Pitch.pid_angle_parameter[1] = 0, gimbal_Pitch.pid_angle_parameter[2] = 3;
    gimbal_Pitch.angle_target = MIN_ANGLE;

    // 初始化pid结构体
    pid_init(&gimbal_Pitch.pid, gimbal_Pitch.pid_parameter, 30000, 3000);
    pid_init(&gimbal_Pitch.pid_angle, gimbal_Pitch.pid_angle_parameter, 30000, 3000);
}

/**
 * @brief 云台电机PID计算并使能电机
 *
 */
static void gimbal_current_give()
{
    gimbal_Pitch.motor_info.set_current = pid_calc(&gimbal_Pitch.pid, gimbal_Pitch.motor_info.rotor_speed, gimbal_Pitch.speed_target);
    set_curruent(MOTOR_6020_0, hcan1, 0, 0, gimbal_Pitch.motor_info.set_current, 0);
}

/**
 * @brief 遥控器控制云台电机
 *
 */
static void Pitch_control()
{
    if (rc_ctrl.rc.ch[1] >= -660 && rc_ctrl.rc.ch[1] <= 660)
    {
        if (recv->is_tracking && (rc_ctrl.rc.s[1] == 3 || rc_ctrl.mouse.press_r)) // if (rc_ctrl.mouse.press_r && recv.is_tracking)
            gimbal_Pitch.angle_target = recv->pitch;
        else
            gimbal_Pitch.angle_target += rc_ctrl.rc.ch[1] / 660.0 * 0.25 + (rc_ctrl.mouse.y / 16384.00 * 55);
        Angle_Limit(&gimbal_Pitch.angle_target);

        gimbal_Pitch.err_angle = gimbal_Pitch.angle_target - gimba_IMU_data->Pitch;
        detel_calc2(&gimbal_Pitch.err_angle);

        gimbal_Pitch.speed_target = gimbal_Pitch_PID_cal(&gimbal_Pitch.pid_angle, 0, gimbal_Pitch.err_angle);
    }
    else
    {
        gimbal_Pitch.speed_target = 0;
    }
}

/**
 * @brief 限制角度
 *
 * @param angle 需要限制的角度
 */
static void Angle_Limit(fp32 *angle)
{
    if (*angle <= MIN_ANGLE && *angle >= 0)
        *angle = MIN_ANGLE;
    else if (*angle >= MAX_ANGLE)
        *angle = MAX_ANGLE;
}

/**
 * @brief 角度差计算
 *
 * @param angle 需要计算的角度
 */
static void detel_calc2(fp32 *angle)
{
    if (*angle > 180)
        *angle -= 360;
}
