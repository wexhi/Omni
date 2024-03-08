#include "struct_typedef.h"
#include "exchange.h"
#include "ins_task.h"
#include "rm_referee.h"
#include "drv_can.h"
extern int16_t up_angle[2]; // 上C的陀螺仪数据
extern int16_t aim_target;	// 上C的云台YAW轴目标，用于自瞄
extern float yaw_aim;		// 云台YAW轴目标，用于自瞄

static referee_info_t *referee_data2up; // 用于获取裁判系统的数据
static uint8_t tx_data[8];			 // 用于发送数据
UP_C_angle_t UP_C_angle;			 // 上C的陀螺仪数据

static void SendToUP(); // 发送数据到上C

void exchange_task()
{
	while (1)
	{
		// 接收并解算上C的陀螺仪数据
		UP_C_angle.yaw = up_angle[0] / 100.0f;
		UP_C_angle.yaw_gyro = up_angle[1] / 2000.0f;

		// 接收并解算上C的云台YAW轴目标，用于自瞄
		yaw_aim = aim_target / 100.0f;

		SendToUP(); // 发送数据到上C

		osDelay(1);
	}
}

void ExchangInit()
{
	referee_data2up = RefereeInit(&huart6); // 裁判系统初始化
}

static void SendToUP()
{
	// 发送数据到上C
	// 发送热量上限
	tx_data[0] = (uint8_t)(referee_data2up->GameRobotState.shooter_id1_17mm_cooling_limit >> 8) & 0xff;
	tx_data[1] = (uint8_t)(referee_data2up->GameRobotState.shooter_id1_17mm_cooling_limit >> 8);
	// 发送17mm枪口热量
	tx_data[2] = (uint8_t)(referee_data2up->PowerHeatData.shooter_heat0 >> 8) & 0xff;
	tx_data[3] = (uint8_t)(referee_data2up->PowerHeatData.shooter_heat0 >> 8);
	// 发送射击模式
	tx_data[4] = referee_data2up->ShootData.shooter_id;
	// 保留位
	tx_data[5] = 0; tx_data[6] = 0; tx_data[7] = 0;

	can_remote(tx_data, 0x55); // 发送数据到上C
}