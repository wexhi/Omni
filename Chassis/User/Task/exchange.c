#include "struct_typedef.h"
#include "exchange.h"
#include "ins_task.h"
extern INS_t INS;
extern int16_t up_angle[3];

ins_data_t ins_data;

UP_C_angle_t UP_C_angle;

void exchange_task()
{
	while (1)
	{
		ins_data.angle[0] = INS.Yaw;
		ins_data.angle[1] = INS.Roll;
		ins_data.angle[2] = INS.Pitch;

		UP_C_angle.yaw = up_angle[0] / 1000.0f;
		UP_C_angle.roll = up_angle[1] / 1000.0f;
		UP_C_angle.pitch = up_angle[2] / 1000.0f;

		osDelay(1);
	}
}