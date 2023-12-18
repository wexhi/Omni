#include "Gimbal_task.h"
#include "cmsis_os.h"
#include "INS_task.h"
#include "exchange.h"
#include "drv_can.h"
#define MAX_SPEED 200
#define MAX_ANGLE 200
#define MIN_ANGLE 140

gimbal_t gimbal_Pitch; // 云台电机信息结构体

extern fp32 INS_angle[3];
extern RC_ctrl_t rc_ctrl; // 遥控器信息结构体

// 云台电机的初始化
static void Gimbal_loop_Init();

// 遥控器控制云台电机
static void RC_Pitch_control();

// 云台电机的任务
static void gimbal_current_give();

static void Angle_Limit(fp32 *angle);
static void detel_calc2(fp32 *angle);

void Gimbal_task(void const *pvParameters)
{
    osDelay(8000);
    Gimbal_loop_Init();
    for (;;)
    {
        RC_Pitch_control();
        gimbal_current_give();
        osDelay(1);
    }
}

// 云台电机的初始化
static void Gimbal_loop_Init()
{
    // 初始化pid参数
    gimbal_Pitch.pid_parameter[0] = 80, gimbal_Pitch.pid_parameter[1] = 0.1, gimbal_Pitch.pid_parameter[2] = 0;
    gimbal_Pitch.pid_angle_parameter[0] = 8, gimbal_Pitch.pid_angle_parameter[1] = 0, gimbal_Pitch.pid_angle_parameter[2] = 0;
    gimbal_Pitch.angle_target = 140;

    // 初始化pid结构体
    pid_init(&gimbal_Pitch.pid, gimbal_Pitch.pid_parameter, 30000, 30000);
    pid_init(&gimbal_Pitch.pid_angle, gimbal_Pitch.pid_angle_parameter, 30000, 30000);
}

// 给电流
static void gimbal_current_give()
{
    gimbal_Pitch.motor_info.set_current = pid_calc(&gimbal_Pitch.pid, gimbal_Pitch.motor_info.rotor_speed, gimbal_Pitch.speed_target);
    set_curruent(MOTOR_6020_0, hcan1, 0, 0, gimbal_Pitch.motor_info.set_current, 0);
}

static void RC_Pitch_control()
{
    // Pitch轴
    // 把头装上再写吧
    if (rc_ctrl.rc.ch[1] >= -660 && rc_ctrl.rc.ch[1] <= 660)
    {
        gimbal_Pitch.angle_target += rc_ctrl.rc.ch[1] / 660.0 * 0.25;

        Angle_Limit(&gimbal_Pitch.angle_target);

        gimbal_Pitch.err_angle = gimbal_Pitch.angle_target - INS_angle[2];
        detel_calc2(&gimbal_Pitch.err_angle);

        gimbal_Pitch.speed_target = gimbal_Pitch_PID_cal(&gimbal_Pitch.pid_angle, 0, gimbal_Pitch.err_angle);

        }
    else
    {
        gimbal_Pitch.speed_target = 0;
    }
}

static void Angle_Limit(fp32 *angle)
{
    if (*angle <= MIN_ANGLE && *angle > 0)
    {
        *angle = MIN_ANGLE;
    }
    else if (*angle >= MAX_ANGLE)
    {
        *angle = MAX_ANGLE;
    }
}

static void detel_calc2(fp32 *angle)
{
    if (*angle > 180)
    {
        *angle -= 360;
    }
}
