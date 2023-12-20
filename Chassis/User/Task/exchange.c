#include "struct_typedef.h"
#include "exchange.h"
#include "ins_task.h"
extern int16_t up_angle[3];

ins_data_t ins_data;

UP_C_angle_t UP_C_angle;

void exchange_task()
{
	while (1)
	{

		UP_C_angle.yaw = up_angle[0] / 100.0f;
		UP_C_angle.roll = up_angle[1] / 100.0f;
		UP_C_angle.pitch = up_angle[2] / 100.0f;

		osDelay(1);
	}
}