#include "struct_typedef.h"
#include "exchange.h"
#include "ins_task.h"
#include "drv_can.h"
extern INS_t INS;

ins_data_t ins_data;

static void Up_send_to_down();

void exchange_task()
{
	for (;;)
	{
		osDelay(1);
		// ins_data.angle[0]=INS.Yaw;
		// ins_data.angle[1]=INS.Roll;
		// ins_data.angle[2]=INS.Pitch;
		Up_send_to_down();
	}
}

//================================================上C向下C发送数据================================================//
static void Up_send_to_down()
{
	uint8_t ins_buf[8];
	ins_buf[0] = ((int16_t)INS.Yaw >> 8) & 0xff;
	ins_buf[1] = (int16_t)INS.Yaw & 0xff;
	ins_buf[2] = ((int16_t)INS.Roll >> 8) & 0xff;
	ins_buf[3] = (int16_t)INS.Roll & 0xff;
	ins_buf[4] = ((int16_t)INS.Pitch >> 8) & 0xff;
	ins_buf[5] = (int16_t)INS.Pitch & 0xff;
	ins_buf[6] = ((int16_t)INS.YawTotalAngle >> 8) & 0xff;
	ins_buf[7] = (int16_t)INS.YawTotalAngle & 0xff;
	can_remote(ins_buf, 0x55);
}