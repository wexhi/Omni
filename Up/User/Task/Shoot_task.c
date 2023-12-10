#include "Shoot_task.h"
#include "cmsis_os.h"
#include "INS_task.h"
#include "exchange.h"
#include "drv_can.h"

shooter_t shooter; // 发射机构信息结构体
// 电机0为拨盘电机，电机1为弹舱盖电机，电机2、3为摩擦轮电机

extern RC_ctrl_t rc_ctrl; // 遥控器信息结构体

static void Shooter_Inint();         // 发射机构的初始化
static void model_choice();          // 模式选择
static void dial_control();          // 拨盘电机控制
static void friction_control();      // 摩擦轮电机控制
static void bay_control();           // 弹舱电机控制
static void shooter_current_given(); // 给电流

void Shoot_task(void const *pvParameters)
{
    Shooter_Inint();
    for (;;)
    {
        model_choice();
        shooter_current_given();
        osDelay(1);
    }
}

// 发射机构的初始化
static void Shooter_Inint(void)
{
    // 初始化pid参数
    shooter.pid_dial_para[0] = 20, shooter.pid_dial_para[1] = 0, shooter.pid_dial_para[2] = 0;
    shooter.pid_friction_para[0] = 30, shooter.pid_friction_para[1] = 0, shooter.pid_friction_para[2] = 0;
    shooter.pid_bay_para[0] = 10, shooter.pid_bay_para[1] = 0, shooter.pid_bay_para[2] = 0;

    // 初始化pid结构体
    pid_init(&shooter.pid_dial, shooter.pid_dial_para, 8000, 8000);
    pid_init(&shooter.pid_friction, shooter.pid_friction_para, 8000, 8000);
    pid_init(&shooter.pid_bay, shooter.pid_bay_para, 8000, 8000);

    // 初始化速度目标
    shooter.dial_speed_target = 0;
    shooter.friction_speed_target[0] = 0, shooter.friction_speed_target[1] = 0;
    shooter.bay_speed_target = 0;
}

// 模式选择
static void model_choice(void)
{
    // 取消注释开始发射
    // friction_control();
    if (rc_ctrl.rc.s[1] == 1)
    {
        // 发射
        // dial_control();
        // bay_control();
    }
    else
    {
        shooter.dial_speed_target = 0;
        shooter.bay_speed_target = 0;
        // 停止
    }
}

// 拨盘电机控制
static void dial_control(void)
{
    if (rc_ctrl.rc.s[1] == 1)
    {
        LEDR_OFF();
        shooter.dial_speed_target = 2000;
    }
    else
    {
        shooter.dial_speed_target = 0;
    }
}

// 摩擦轮电机控制
static void friction_control(void)
{
    shooter.friction_speed_target[0] = -2000;
    shooter.friction_speed_target[1] = 2000;
}

// 弹舱电机控制
static void bay_control(void)
{
    // 暂留
    shooter.bay_speed_target = 0;
}

// 给电流
static void shooter_current_given(void)
{
    shooter.motor_info[0].set_current = pid_calc(&shooter.pid_dial, shooter.motor_info[0].rotor_speed, shooter.dial_speed_target);            // 拨盘电机
    shooter.motor_info[1].set_current = pid_calc(&shooter.pid_bay, shooter.motor_info[1].rotor_speed, shooter.bay_speed_target);              // 弹舱电机
    shooter.motor_info[2].set_current = pid_calc(&shooter.pid_friction, shooter.motor_info[2].rotor_speed, shooter.friction_speed_target[0]); // 摩擦轮电机
    shooter.motor_info[3].set_current = pid_calc(&shooter.pid_friction, shooter.motor_info[3].rotor_speed, shooter.friction_speed_target[1]); // 摩擦轮电机
    // set_motor_current_shoot(1, shooter.motor_info[0].set_current, shooter.motor_info[1].set_current, shooter.motor_info[2].set_current, shooter.motor_info[3].set_current);
    set_curruent(MOTOR_3508_1, hcan1, shooter.motor_info[0].set_current, shooter.motor_info[1].set_current, shooter.motor_info[2].set_current, shooter.motor_info[3].set_current);
}