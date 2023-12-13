#include "struct_typedef.h"
#include "exchange.h"
#include "INS_task.h"
#include "drv_can.h"
#include <string.h>

extern fp32 INS_angle[3];
int16_t INS_angle_send[3];
uint8_t ins_buf[8];

static void Up_send_to_down();

void exchange_task()
{
	for (;;)
	{
		osDelay(1);
		Up_send_to_down();
	}
}

//================================================上C向下C发送数据================================================//
static void Up_send_to_down()
{
	INS_angle_send[0] = INS_angle[0] * 100;
	INS_angle_send[1] = INS_angle[1] * 100;
	INS_angle_send[2] = INS_angle[2] * 100;

	ins_buf[0] = (INS_angle_send[0] >> 8) & 0xff;
	ins_buf[1] = INS_angle_send[1] >> 8;
	ins_buf[2] = (INS_angle_send[2] >> 8) & 0xff;
	ins_buf[3] = INS_angle_send[3] >> 8;
	ins_buf[4] = (INS_angle_send[4] >> 8) & 0xff;
	ins_buf[5] = INS_angle_send[5] >> 8;
	ins_buf[6] = 0;
	ins_buf[7] = 0;
	can_remote(ins_buf, 0x55);
}