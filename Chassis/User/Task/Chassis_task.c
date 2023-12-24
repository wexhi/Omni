#include "Chassis_task.h"
#include "cmsis_os.h"
#include "INS_task.h"
#include "exchange.h"
#include "drv_can.h"
#include "arm_math.h"

#define RC_MAX 660
#define RC_MIN -660
#define motor_max 2000
#define motor_min -2000
#define Wz_max 3000
#define angle_valve 5
#define angle_weight 60

chassis_t chassis;

pid_struct_t supercap_pid;
motor_info_t motor_info_chassis[10]; // 电机信息结构体
fp32 superpid[3] = {120, 0.1, 0};

int8_t chassis_mode;

extern RC_ctrl_t rc_ctrl; // 遥控器信息结构体
extern float powerdata[4];
extern UP_C_angle_t UP_C_angle;
extern INS_t INS;

uint8_t rc[18];

static void Chassis_Init();

static void Chassis_loop_Init();

// 模式选择
static void mode_chooce();

// 遥控器控制底盘电机
static void RC_Move(void);

// 小陀螺模式
static void gyroscope(void);

// 速度限制函数
static void Motor_Speed_limiting(volatile int16_t *motor_speed, int16_t limit_speed);

// 电机电流控制
static void chassis_current_give();

// 运动解算
static void chassis_motol_speed_calculate();

// 底盘跟随云台
static void chassis_follow_gimbal();

// 获取上下C板角度差
static void get_UpDown_Err();

// 旋转矩阵
static void rotate();

void Chassis_task(void const *pvParameters)
{
  Chassis_Init();

  for (;;) // 底盘运动任务
  {
    Chassis_loop_Init();
    // 获取上下C板角度差
    get_UpDown_Err();

    // 选择底盘运动模式
    mode_chooce();

    // 电机速度解算
    chassis_motol_speed_calculate();

    // 电机电流控制
    chassis_current_give();
    osDelay(1);
  }
}

static void Chassis_Init()
{
  chassis.pid_parameter[0] = 30, chassis.pid_parameter[1] = 0.5, chassis.pid_parameter[2] = 0;

  for (uint8_t i = 0; i < 4; i++)
  {
    pid_init(&chassis.pid[i], chassis.pid_parameter, 16384, 16384); // init pid parameter, kp=40, ki=3, kd=0, output limit = 16384
  }
  pid_init(&supercap_pid, superpid, 3000, 3000); // init pid parameter, kp=40, ki=3, kd=0, output limit = 16384

  chassis.Vx = 0, chassis.Vy = 0, chassis.Wz = 0;

  chassis.imu_err = 0;
}

static void Chassis_loop_Init()
{
  chassis.Vx = 0;
  chassis.Vy = 0;
  chassis.Wz = 0;
}

static void mode_chooce()
{
  // 遥控器控制
  // chanel 0 left max==-660,right max==660
  // chanel 1 up max==660,down max==-660
  // chanel 2 left max==-660,right max==660
  // chanel 3 up max==660,down max==-660
  // chanel 4 The remote control does not have this channel

  if (rc_ctrl.rc.s[0] == 1)
  {
    LEDB_ON(); // BLUE LED
    LEDR_OFF();
    LEDG_OFF();
    // gyroscope();
  }
  else if (rc_ctrl.rc.s[0] == 2)
  {
    LEDG_ON(); // GREEN LED
    LEDR_OFF();
    LEDB_OFF();
    chassis_follow_gimbal();
  }
  else if (rc_ctrl.rc.s[0] == 3)
  {
    LEDR_ON(); // RED LED
    LEDB_OFF();
    LEDG_OFF();
    RC_Move();
  }
  else
  {
    LEDR_OFF();
    LEDB_OFF();
    LEDG_OFF();
  }
}

// 运动解算
static void chassis_motol_speed_calculate()
{

  // 根据分解的速度调整电机速度目标
  chassis.speed_target[CHAS_LF] = -chassis.Wz + sinf(PI / 4) * chassis.Vx - sinf(PI / 4) * chassis.Vy; // 1
  chassis.speed_target[CHAS_RF] = -chassis.Wz - sinf(PI / 4) * chassis.Vx - sinf(PI / 4) * chassis.Vy; // 2
  chassis.speed_target[CHAS_RB] = -chassis.Wz - sinf(PI / 4) * chassis.Vx + sinf(PI / 4) * chassis.Vy; // 3
  chassis.speed_target[CHAS_LB] = -chassis.Wz + sinf(PI / 4) * chassis.Vx + sinf(PI / 4) * chassis.Vy; // 4
}
// 运动解算
// 速度限制函数
static void Motor_Speed_limiting(volatile int16_t *motor_speed, int16_t limit_speed)
{
  uint8_t i = 0;
  int16_t max = 0;
  int16_t temp = 0;
  int16_t max_speed = limit_speed;
  fp32 rate = 0;
  for (i = 0; i < 4; i++)
  {
    temp = (motor_speed[i] > 0) ? (motor_speed[i]) : (-motor_speed[i]); // 求绝对值

    if (temp > max)
    {
      max = temp;
    }
  }

  if (max > max_speed)
  {
    rate = max_speed * 1.0 / max; //*1.0转浮点类型，不然可能会直接得0   *1.0 to floating point type, otherwise it may directly get 0
    for (i = 0; i < 4; i++)
    {
      motor_speed[i] *= rate;
    }
  }
}

// 电机电流控制
static void chassis_current_give()
{

  uint8_t i = 0;

  for (i = 0; i < 4; i++)
  {
    chassis.motor_info[i].set_current = pid_calc(&chassis.pid[i], chassis.motor_info[i].rotor_speed, chassis.speed_target[i]);
  }
  set_motor_current_chassis(0, chassis.motor_info[0].set_current, chassis.motor_info[1].set_current, chassis.motor_info[2].set_current, chassis.motor_info[3].set_current);
  // set_curruent(MOTOR_3508_0, hcan1, chassis.motor_info[0].set_current, chassis.motor_info[1].set_current, chassis.motor_info[2].set_current, chassis.motor_info[3].set_current);
}

// 线性映射函数
static int16_t map_range(int value, int from_min, int from_max, int to_min, int to_max)
{
  // 首先将输入值映射到[0, 1]的范围
  double normalized_value = (value * 1.0 - from_min * 1.0) / (from_max * 1.0 - from_min * 1.0);

  // 然后将[0, 1]的范围映射到[to_min, to_max]的范围
  int16_t mapped_value = (int16_t)(normalized_value * (to_max - to_min) + to_min);

  return mapped_value;
}

static void RC_Move(void)
{
  // 从遥控器获取控制输入
  chassis.Vx = rc_ctrl.rc.ch[3]; // 前后输入
  chassis.Vy = rc_ctrl.rc.ch[2]; // 左右输入
  chassis.Wz = rc_ctrl.rc.ch[4]; // 旋转输入

  rotate();

  /*************记得加上线性映射***************/
  chassis.Vx = map_range(chassis.Vx, RC_MIN, RC_MAX, motor_min, motor_max);
  chassis.Vy = map_range(chassis.Vy, RC_MIN, RC_MAX, motor_min, motor_max);
  chassis.Wz = map_range(chassis.Wz, RC_MIN, RC_MAX, motor_min, motor_max);
}

// 小陀螺模式
static void gyroscope(void)
{
  chassis.Wz = 900;
}

// 底盘跟随云台
static void chassis_follow_gimbal()
{
  if (chassis.err_angle > angle_valve || chassis.err_angle < -angle_valve)
  {
    chassis.Wz -= chassis.err_angle * angle_weight;
  }

  if (chassis.Wz > Wz_max)
  {
    chassis.Wz = Wz_max;
  }
  else if (chassis.Wz < -Wz_max)
  {
    chassis.Wz = -Wz_max;
  }
  // 从遥控器获取控制输入
  chassis.Vx = rc_ctrl.rc.ch[3]; // 前后输入
  chassis.Vy = rc_ctrl.rc.ch[2]; // 左右输入
  chassis.Vx = map_range(chassis.Vx, RC_MIN, RC_MAX, motor_min, motor_max);
  chassis.Vy = map_range(chassis.Vy, RC_MIN, RC_MAX, motor_min, motor_max);
}

// 获取上下C板角度差
static void get_UpDown_Err()
{
  if (rc_ctrl.rc.s[0] == 1)
  {
    chassis.imu_err = -chassis.err_angle;
  }

  chassis.err_angle = INS.Yaw - UP_C_angle.yaw + chassis.imu_err;

  // 越界处理,保证转动方向不变
  if (chassis.err_angle < -180) //	越界时：180 -> -180
  {
    chassis.err_angle += 360;
  }

  else if (chassis.err_angle > 180) //	越界时：-180 -> 180
  {
    chassis.err_angle -= 360;
  }

  chassis.err_angle_rad = chassis.err_angle / 57.3;
}

// 旋转矩阵
static void rotate()
{
  int16_t temp_Vx = 0;
  int16_t temp_Vy = 0;
  temp_Vx = chassis.Vx * cosf(chassis.err_angle_rad) - chassis.Vy * sinf(chassis.err_angle_rad);
  temp_Vy = chassis.Vx * sinf(chassis.err_angle_rad) + chassis.Vy * cosf(chassis.err_angle_rad);
  chassis.Vx = temp_Vx;
  chassis.Vy = temp_Vy;
}