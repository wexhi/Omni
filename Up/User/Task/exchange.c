#include "struct_typedef.h"
#include "exchange.h"
#include "INS_task.h"
#include "drv_can.h"
#include <string.h>


int16_t yaw_to_down = 0;
int16_t pitch_to_down = 0;
int16_t roll_to_down = 0;
int16_t yaw_total_angle_to_down = 0;

extern fp32 INS_angle[3];

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
	uint8_t ins_buf[8] = {0};
	ins_buf[0] = 8;						   //	imu头帧标识
	memcpy(&ins_buf[1], &INS_angle[0], 4); // 获取yaw的角度并储存在发送的字节中
	can_remote(ins_buf, 0x55);
}