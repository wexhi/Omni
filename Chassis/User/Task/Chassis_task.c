#include "Chassis_task.h"
#include "cmsis_os.h"
#include "INS_task.h"
#include "exchange.h"
#include "drv_can.h"
#include "arm_math.h"
#include "rm_referee.h"
#include "referee_protocol.h"

#define CHASSIS_MAX_SPEED 4000
#define RC_OFFSET CHASSIS_MAX_SPEED / 660
#define KEY_MAX 4000
#define WZ_MAX 4000
#define ANGLE_VALVE 5
#define ANGLE_WEIGHT 60
#define KEY_START_OFFSET 15
#define KEY_STOP_OFFSET 30

chassis_t chassis;                   // 底盘信息结构体
pid_struct_t supercap_pid;           // 超级电容PID结构体
fp32 superpid[3] = {120, 0.1, 0};    // 超级电容PID参数
int8_t chassis_mode;                 // 底盘模式
static attitude_t *chassis_IMU_data; // 底盘IMU数据
static referee_info_t *referee_data; // 用于获取裁判系统的数据
// 功率限制算法的变量定义
float Watch_Power_Max;                                                 // 限制值
float Watch_Power;                                                     // 实时功率
uint16_t Watch_Buffer;                                                    // 缓冲能量值
double Chassis_pidout;                                                 // 输出值
double Chassis_pidout_target;                                          // 目标值
static double Scaling1 = 0, Scaling2 = 0, Scaling3 = 0, Scaling4 = 0;  // 比例
float Klimit = 1;                                                      // 限制值
float Plimit = 0;                                                      // 约束比例
float Chassis_pidout_max;                                              // 输出值限制
static int16_t key_x_fast, key_y_fast, key_x_slow, key_y_slow, key_Wz; // 键盘控制变量

extern RC_ctrl_t rc_ctrl;            // 遥控器信息结构体
extern float powerdata[4];           // 电源数据
extern UP_C_angle_t UP_C_angle;      // 上C的陀螺仪数据
extern ext_power_heat_data_t powerd; // 电源数据

static void Chassis_loop_Init();                                                      // 底盘循环初始化
static void mode_chooce();                                                            // 模式选择
static void GimbalMove(void);                                                         // 遥控器控制底盘电机
static void Motor_Speed_limiting(volatile int16_t *motor_speed, int16_t limit_speed); // 速度限制函数
static void chassis_current_give();                                                   // 电机电流控制
static void chassis_motol_speed_calculate();                                          // 运动解算
static void chassis_follow_gimbal();                                                  // 底盘跟随云台
static void get_UpDown_Err();                                                         // 获取上下C板角度差
static void rotate();                                                                 // 旋转矩阵
static void key_control(void);                                                        // 键盘控制函数
static void Chassis_Power_Limit(double Chassis_pidout_target_limit);                  // 底盘功率限制

void Chassis_task(void const *pvParameters)
{
  for (;;) // 底盘运动任务
  {
    Chassis_loop_Init();             // 获取上下C板角度差
    get_UpDown_Err();                // 获取上下C板角度差
    mode_chooce();                   // 模式选择
    chassis_motol_speed_calculate(); // 运动解算
    chassis_current_give();          // 电机电流控制
    osDelay(1);
  }
}

/**
 * @brief 底盘初始化，初始化底盘电机PID参数，将底盘速度清零，将IMU误差清零
 *
 * @todo 加入超级电容PID参数初始化
 */
void Chassis_Init()
{
  chassis_IMU_data = INS_Init();       // 底盘IMU初始化
  referee_data = RefereeInit(&huart6); // 裁判系统初始化

  chassis.pid_parameter[0] = 30,
  chassis.pid_parameter[1] = 0.5, chassis.pid_parameter[2] = 0; // 底盘电机PID参数

  for (uint8_t i = 0; i < 4; i++)
  {
    pid_init(&chassis.pid[i], chassis.pid_parameter, 16384, 16384); // 底盘电机PID参数初始化
  }
  pid_init(&supercap_pid, superpid, 3000, 3000); // 超级电容PID参数初始化

  chassis.Vx = 0, chassis.Vy = 0, chassis.Wz = 0; // 底盘速度清零
  chassis.imu_err = 0;                            // IMU误差清零
}

/**
 * @brief 底盘循环初始化，将底盘速度清零,防止疯车
 *
 */
static void Chassis_loop_Init()
{
  chassis.Vx = 0; // 底盘速度清零
  chassis.Vy = 0;
  chassis.Wz = 0;
}

/**
 * @brief 底盘模式选择，根据遥控器的输入选择底盘的运动模式
 *        模式1： 默认为紧急停车模式 （蓝灯）
 *       模式2： 底盘跟随云台模式 （绿灯）
 *      模式3： 遥控器控制底盘模式 （红灯）
 *
 */
static void mode_chooce()
{
  // 遥控器控制
  // chanel 0 left max==-660,right max==660
  // chanel 1 up max==660,down max==-660
  // chanel 2 left max==-660,right max==660
  // chanel 3 up max==660,down max==-660
  // chanel 4 The remote control does not have this channel

  if (rc_ctrl.rc.s[0] == 1) // 右侧拨杆向上
  {
    LEDB_ON(); // BLUE LED
    LEDR_OFF();
    LEDG_OFF();
    Chassis_loop_Init(); // 底盘速度清零
  }
  else if (rc_ctrl.rc.s[0] == 2) // 右侧拨杆向下
  {
    LEDG_ON(); // GREEN LED
    LEDR_OFF();
    LEDB_OFF();
    chassis_follow_gimbal(); // 底盘跟随云台
  }
  else if (rc_ctrl.rc.s[0] == 3) // 右侧拨杆中间
  {
    LEDR_ON(); // RED LED
    LEDB_OFF();
    LEDG_OFF();
    key_control(); // 键盘控制
    GimbalMove();  // 遥控器控制底盘
  }
  else
  {
    LEDR_OFF();
    LEDB_OFF();
    LEDG_OFF();
  }
}

/**
 * @brief 底盘电机速度解算，根据底盘速度分解的速度调整电机速度目标
 *
 */
static void chassis_motol_speed_calculate()
{

  // 根据分解的速度调整电机速度目标
  chassis.speed_target[CHAS_LF] = -chassis.Wz + sinf(PI / 4) * chassis.Vx - sinf(PI / 4) * chassis.Vy; // 1
  chassis.speed_target[CHAS_RF] = -chassis.Wz - sinf(PI / 4) * chassis.Vx - sinf(PI / 4) * chassis.Vy; // 2
  chassis.speed_target[CHAS_RB] = -chassis.Wz - sinf(PI / 4) * chassis.Vx + sinf(PI / 4) * chassis.Vy; // 3
  chassis.speed_target[CHAS_LB] = -chassis.Wz + sinf(PI / 4) * chassis.Vx + sinf(PI / 4) * chassis.Vy; // 4
}

/**
 * @brief 速度限制函数，限制底盘速度
 *
 * @param motor_speed 4个底盘电机目标速度的数组
 * @param limit_speed 限制的最大速度
 */
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
      max = temp;
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

/**
 * @brief 电机电流控制，根据PID算法计算电机电流，使能电机
 *
 */
static void chassis_current_give()
{

  Motor_Speed_limiting(chassis.speed_target, CHASSIS_MAX_SPEED); // 限制最大期望速度，输入参数是限制速度值(同比缩放)
  uint8_t i = 0;

  for (i = 0; i < 4; i++)
  {
    chassis.motor_info[i].set_current = pid_calc(&chassis.pid[i], chassis.motor_info[i].rotor_speed, chassis.speed_target[i]);
  }
  Chassis_Power_Limit(CHASSIS_MAX_SPEED * 4); // 限制底盘功率
  set_motor_current_chassis(0, chassis.motor_info[0].set_current, chassis.motor_info[1].set_current, chassis.motor_info[2].set_current, chassis.motor_info[3].set_current);
}

/**
 * @brief 遥控器控制底盘，后续考虑只保留此模式
 *
 */
static void GimbalMove(void)
{
  // 从遥控器和键盘获取控制输入
  chassis.Vx = rc_ctrl.rc.ch[3] * RC_OFFSET + key_x_fast - key_x_slow; // 前后输入
  chassis.Vy = rc_ctrl.rc.ch[2] * RC_OFFSET + key_y_fast - key_y_slow; // 左右输入
  chassis.Wz = rc_ctrl.rc.ch[4] * RC_OFFSET + key_Wz;                  // 旋转输入

  if (chassis.Wz > WZ_MAX / 10) // 旋转速度限制
  {
    chassis.Vx = chassis.Vx / 2;
    chassis.Vy = chassis.Vy / 2;
  }

  rotate(); // 旋转矩阵
}

/**
 * @brief 底盘跟随云台
 *
 */
static void chassis_follow_gimbal()
{
  if (chassis.err_angle > ANGLE_VALVE || chassis.err_angle < -ANGLE_VALVE) // 云台偏差大于5度
    chassis.Wz -= chassis.err_angle * ANGLE_WEIGHT;                        // 旋转速度输入

  if (chassis.Wz > WZ_MAX) // 旋转速度限制
    chassis.Wz = WZ_MAX;
  else if (chassis.Wz < -WZ_MAX)
    chassis.Wz = -WZ_MAX;

  // 从遥控器获取控制输入
  chassis.Vx = rc_ctrl.rc.ch[3] * RC_OFFSET; // 前后输入
  chassis.Vy = rc_ctrl.rc.ch[2] * RC_OFFSET; // 左右输入
}

/**
 * @brief 获取上下C板角度差
 *
 * @todo 添加键盘控制IMU误差修正
 */
static void get_UpDown_Err()
{
  if (rc_ctrl.rc.s[0] == 1 || x_flag) // 修正IMU误差
    chassis.imu_err = chassis_IMU_data->Yaw - UP_C_angle.yaw;
  else
    chassis.err_angle = chassis_IMU_data->Yaw - UP_C_angle.yaw - chassis.imu_err; // 获取上下C板角度差

  if (chassis.Wz > 60)
    chassis.err_angle -= 0.02f;
  if (chassis.Wz < -60)
    chassis.err_angle += 0.02f;

  // 越界处理,保证转动方向不变
  if (chassis.err_angle < -180) //	越界时：180 -> -180
    chassis.err_angle += 360;
  else if (chassis.err_angle > 180) //	越界时：-180 -> 180
    chassis.err_angle -= 360;

  chassis.err_angle_rad = chassis.err_angle / 57.3f; // 角度转弧度
}

/**
 * @brief 旋转矩阵
 *
 */
static void rotate()
{
  int16_t temp_Vx = 0;
  int16_t temp_Vy = 0;
  temp_Vx = chassis.Vx * cosf(chassis.err_angle_rad) - chassis.Vy * sinf(chassis.err_angle_rad);
  temp_Vy = chassis.Vx * sinf(chassis.err_angle_rad) + chassis.Vy * cosf(chassis.err_angle_rad);
  chassis.Vx = temp_Vx;
  chassis.Vy = temp_Vy;
}

/**
 * @brief 键盘控制
 *
 * @todo 写的太丑陋了，后续简洁一点
 */
static void key_control(void)
{
  if (d_flag)
    key_y_fast += KEY_START_OFFSET;
  else
    key_y_fast -= KEY_STOP_OFFSET;
  if (a_flag)
    key_y_slow += KEY_START_OFFSET;
  else
    key_y_slow -= KEY_STOP_OFFSET;
  if (w_flag)
    key_x_fast += KEY_START_OFFSET;
  else
    key_x_fast -= KEY_STOP_OFFSET;
  if (s_flag)
    key_x_slow += KEY_START_OFFSET;
  else
    key_x_slow -= KEY_STOP_OFFSET;
  if (shift_flag)
    key_Wz += KEY_START_OFFSET;
  else
    key_Wz -= KEY_STOP_OFFSET;

  if (key_x_fast > KEY_MAX)
    key_x_fast = KEY_MAX;
  if (key_x_fast < 0)
    key_x_fast = 0;
  if (key_x_slow > KEY_MAX)
    key_x_slow = KEY_MAX;
  if (key_x_slow < 0)
    key_x_slow = 0;
  if (key_y_fast > KEY_MAX)
    key_y_fast = KEY_MAX;
  if (key_y_fast < 0)
    key_y_fast = 0;
  if (key_y_slow > KEY_MAX)
    key_y_slow = KEY_MAX;
  if (key_y_slow < 0)
    key_y_slow = 0;
  if (key_Wz > KEY_MAX)
    key_Wz = KEY_MAX;
  if (key_Wz < 0)
    key_Wz = 0;
}

static void Chassis_Power_Limit(double Chassis_pidout_target_limit)
{
  // 819.2/A，假设最大功率为120W，那么能够通过的最大电流为5A，取一个保守值：800.0 * 5 = 4000

  Watch_Power_Max = Klimit;
  Watch_Power = referee_data->PowerHeatData.chassis_power; // powerd.chassis_power
  Watch_Buffer = referee_data->PowerHeatData.buffer_energy; // powerd.chassis_power_buffer
  // Hero_chassis_power_buffer;//限制值，功率值，缓冲能量值，初始值是1，0，0
  // get_chassis_power_and_buffer(&Power, &Power_Buffer, &Power_Max);//通过裁判系统和编码器值获取（限制值，实时功率，实时缓冲能量）

  Chassis_pidout_max = 61536; // 32768，40，960			15384 * 4，取了4个3508电机最大电流的一个保守值

  if (Watch_Power > 600)
    Motor_Speed_limiting(chassis.speed_target, 4096); // 限制最大速度 ;//5*4*24;先以最大电流启动，后平滑改变，不知道为啥一开始用的Power>960,可以观测下这个值，看看能不能压榨缓冲功率
  else
  {
    Chassis_pidout = (fabs(chassis.speed_target[0] - chassis.motor_info[0].rotor_speed) +
                      fabs(chassis.speed_target[1] - chassis.motor_info[1].rotor_speed) +
                      fabs(chassis.speed_target[2] - chassis.motor_info[2].rotor_speed) +
                      fabs(chassis.speed_target[3] - chassis.motor_info[3].rotor_speed)); // fabs是求绝对值，这里获取了4个轮子的差值求和

    //	Chassis_pidout_target = fabs(motor_speed_target[0]) + fabs(motor_speed_target[1]) + fabs(motor_speed_target[2]) + fabs(motor_speed_target[3]);

    /*期望滞后占比环，增益个体加速度*/
    if (Chassis_pidout)
    {
      Scaling1 = (chassis.speed_target[0] - chassis.motor_info[0].rotor_speed) / Chassis_pidout;
      Scaling2 = (chassis.speed_target[1] - chassis.motor_info[1].rotor_speed) / Chassis_pidout;
      Scaling3 = (chassis.speed_target[2] - chassis.motor_info[2].rotor_speed) / Chassis_pidout;
      Scaling4 = (chassis.speed_target[3] - chassis.motor_info[3].rotor_speed) / Chassis_pidout; // 求比例，4个scaling求和为1
    }
    else
    {
      Scaling1 = 0.25, Scaling2 = 0.25, Scaling3 = 0.25, Scaling4 = 0.25;
    }

    /*功率满输出占比环，车总增益加速度*/
    //		if(Chassis_pidout_target) Klimit=Chassis_pidout/Chassis_pidout_target;	//375*4 = 1500
    //		else{Klimit = 0;}
    Klimit = Chassis_pidout / Chassis_pidout_target_limit;

    if (Klimit > 1)
      Klimit = 1;
    else if (Klimit < -1)
      Klimit = -1; // 限制绝对值不能超过1，也就是Chassis_pidout一定要小于某个速度值，不能超调

    /*缓冲能量占比环，总体约束*/
    if (Watch_Buffer < 50 && Watch_Buffer >= 40)
      Plimit = 0.9; // 近似于以一个线性来约束比例（为了保守可以调低Plimit，但会影响响应速度）
    else if (Watch_Buffer < 40 && Watch_Buffer >= 35)
      Plimit = 0.75;
    else if (Watch_Buffer < 35 && Watch_Buffer >= 30)
      Plimit = 0.5;
    else if (Watch_Buffer < 30 && Watch_Buffer >= 20)
      Plimit = 0.25;
    else if (Watch_Buffer < 20 && Watch_Buffer >= 10)
      Plimit = 0.125;
    else if (Watch_Buffer < 10 && Watch_Buffer > 0)
      Plimit = 0.05;
    else
    {
      Plimit = 1;
    }

    chassis.motor_info[0].set_current = Scaling1 * (Chassis_pidout_max * Klimit) * Plimit; // 输出值
    chassis.motor_info[1].set_current = Scaling2 * (Chassis_pidout_max * Klimit) * Plimit;
    chassis.motor_info[2].set_current = Scaling3 * (Chassis_pidout_max * Klimit) * Plimit;
    chassis.motor_info[3].set_current = Scaling4 * (Chassis_pidout_max * Klimit) * Plimit; /*同比缩放电流*/
  }
}