#include "struct_typedef.h"
#include "exchange.h"
#include "ins_task.h"
extern INS_t INS;
extern int16_t up_angle[2];
extern int16_t aim_target;
extern float yaw_aim;

ins_data_t ins_data;

UP_C_angle_t UP_C_angle;

void exchange_task()
{
	while (1)
	{
		ins_data.angle[0] = INS.Yaw;
		ins_data.angle[1] = INS.Roll;
		ins_data.angle[2] = INS.Pitch;

		UP_C_angle.yaw = up_angle[0] / 100.0f;
		UP_C_angle.roll = up_angle[1] / 100.0f;

		yaw_aim = aim_target / 100.0f;

		osDelay(1);
	}
}