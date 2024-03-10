#include "Shoot_task.h"
#include "cmsis_os.h"
#include "INS_task.h"
#include "exchange.h"
#include "drv_can.h"

#define MAX_DIAL_SPEED 2000
#define MAX_FRICTION_SPEED 7600
#define KEY_ENTER_OFFSET 10
#define KEY_SLOW_OFFSET 100

shooter_t shooter; // 发射机构信息结构体
// 电机0为拨盘电机，电机1、2为摩擦轮电机，电机3原为弹舱电机，现为备用电机

extern RC_ctrl_t rc_ctrl;      // 遥控器信息结构体
static uint8_t friction_flag;  // 摩擦轮电机开关
static int16_t key_dial_speed; // 键盘控制拨盘电机的速度

static void model_choice();          // 模式选择
static void dial_control();          // 拨盘电机控制
static void friction_control();      // 摩擦轮电机控制
static void bay_control();           // 弹舱电机控制
static void shooter_current_given(); // 给电流
static void GetKeyBoard();           // 获取键盘信息

void Shoot_task(void const *pvParameters)
{
    Shooter_Inint();
    for (;;)
    {
        model_choice();
        shooter_current_given(); // 发射的时候再打开
        osDelay(1);
    }
}

// 发射机构的初始化
void Shooter_Inint(void)
{
    HAL_TIM_Base_Start(&htim1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

    // 初始化pid参数
    shooter.pid_dial_para[0] = 20, shooter.pid_dial_para[1] = 0, shooter.pid_dial_para[2] = 0;
    shooter.pid_friction_para[0] = 30, shooter.pid_friction_para[1] = 0.1, shooter.pid_friction_para[2] = 0;
    shooter.pid_bay_para[0] = 10, shooter.pid_bay_para[1] = 0, shooter.pid_bay_para[2] = 0;

    // 初始化pid结构体
    pid_init(&shooter.pid_dial, shooter.pid_dial_para, 16384, 16384);
    pid_init(&shooter.pid_friction, shooter.pid_friction_para, 16384, 16384);
    pid_init(&shooter.pid_bay, shooter.pid_bay_para, 16384, 16384);

    // 初始化速度目标
    shooter.dial_speed_target = 0;
    shooter.friction_speed_target[0] = 0, shooter.friction_speed_target[1] = 0;
    shooter.bay_speed_target = 0;
}

// 模式选择
static void model_choice(void)
{
    GetKeyBoard();
    bay_control();
    // 取消注释开始发射
    if (rc_ctrl.rc.s[1] == 3 || rc_ctrl.rc.s[1] == 1 || friction_flag == 1)
    {
        // 发射
        friction_control();
        dial_control();
    }
    else
    {
        // 停止
        shooter.friction_speed_target[0] = 0;
        shooter.friction_speed_target[1] = 0;
        shooter.dial_speed_target = 0;
        shooter.bay_speed_target = 0;
    }
}

// 拨盘电机控制
static void dial_control(void)
{
    if (rc_ctrl.rc.s[1] == 1 || rc_ctrl.mouse.press_l == 1 
    && shooter.shoot_heat_limit > shooter.shoot_heat + 30) // 鼠标左键按下发弹
    {
        LEDR_ON();
        LEDB_OFF();
        LEDG_OFF();
        shooter.dial_speed_target = -MAX_DIAL_SPEED;
    }
    else if (f_flag) // 按下f键，拨盘电机反转
    {
        LEDR_OFF();
        LEDB_ON();
        LEDG_OFF();
        shooter.dial_speed_target = key_dial_speed;
    }
    else // 其他情况，拨盘电机停转
    {
        LEDR_OFF();
        LEDB_OFF();
        LEDG_ON();
        shooter.dial_speed_target = 0;
    }
}

// 摩擦轮电机控制
static void friction_control(void)
{
    shooter.friction_speed_target[0] = -MAX_FRICTION_SPEED;
    shooter.friction_speed_target[1] = MAX_FRICTION_SPEED;
}

// 弹舱电机控制
static void bay_control(void)
{
    if (rc_ctrl.rc.s[1] == 2 && !friction_flag)
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 500); // 500 关
    else
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 2100); // 2100 开
}

// 给电流
static void shooter_current_given(void)
{
    shooter.motor_info[0].set_current = pid_calc(&shooter.pid_dial, shooter.motor_info[0].rotor_speed, shooter.dial_speed_target);            // 拨盘电机
    shooter.motor_info[1].set_current = pid_calc(&shooter.pid_friction, shooter.motor_info[1].rotor_speed, shooter.friction_speed_target[0]); // 摩擦轮电机
    shooter.motor_info[2].set_current = pid_calc(&shooter.pid_friction, shooter.motor_info[2].rotor_speed, shooter.friction_speed_target[1]); // 摩擦轮电机
    set_curruent(MOTOR_3508_0, hcan1, shooter.motor_info[0].set_current, shooter.motor_info[1].set_current, shooter.motor_info[2].set_current, 0);
}

static void GetKeyBoard()
{
    if (q_flag) // 摩擦轮电机动
        friction_flag = 1;
    else if (e_flag) // 摩擦轮电机停
        friction_flag = 0;
    if (f_flag) // 拨盘电机反转，防止卡弹
        key_dial_speed += KEY_ENTER_OFFSET;
    else
        key_dial_speed -= KEY_SLOW_OFFSET;
    if (key_dial_speed > MAX_DIAL_SPEED / 2)
        key_dial_speed = MAX_DIAL_SPEED / 2;
    else if (key_dial_speed < 0) // 防止溢出
        key_dial_speed = 0;
}