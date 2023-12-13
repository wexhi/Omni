#include "struct_typedef.h"
#include "exchange.h"
#include "INS_task.h"
#include "drv_can.h"
#include <string.h>

extern fp32 INS_angle[3];
uint8_t ins_buf[8];

static void Up_send_to_down();

void exchange_task()
{
	for (;;)
	{
		osDelay(7);
		Up_send_to_down();
	}
}

//================================================上C向下C发送数据================================================//
static void Up_send_to_down()
{
	ins_buf[0] = (uint16_t)(INS_angle[0] * 100) >> 8;
	ins_buf[1] = (uint16_t)(INS_angle[0] * 100);
	ins_buf[2] = (uint16_t)(INS_angle[1] * 100) >> 8;
	ins_buf[3] = (uint16_t)(INS_angle[1] * 100);
	ins_buf[4] = (uint16_t)(INS_angle[2] * 100) >> 8;
	ins_buf[5] = (uint16_t)(INS_angle[2] * 100);
	ins_buf[6] = 0;
	ins_buf[7] = 0;
	can_remote(ins_buf, 0x55);
}