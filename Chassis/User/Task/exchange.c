#include "struct_typedef.h"
#include "exchange.h"
#include "ins_task.h"
extern int16_t up_angle[2]; // 上C的陀螺仪数据
extern int16_t aim_target;	// 上C的云台YAW轴目标，用于自瞄
extern float yaw_aim;		// 云台YAW轴目标，用于自瞄

ins_data_t ins_data; // 用于传输给上C的数据

UP_C_angle_t UP_C_angle; // 上C的陀螺仪数据

void exchange_task()
{
	while (1)
	{
		// 接收并解算上C的陀螺仪数据
		UP_C_angle.yaw = up_angle[0] / 100.0f;
		UP_C_angle.yaw_gyro = up_angle[1] / 2000.0f;

		// 接收并解算上C的云台YAW轴目标，用于自瞄
		yaw_aim = aim_target / 100.0f;

		osDelay(1);
	}
}