#include "struct_typedef.h"
#include "exchange.h"
#include "INS_task.h"
#include "drv_can.h"
#include <string.h>
#include "miniPC_process.h"

static Vision_Recv_s *vision_recv_data; // 视觉接收数据指针,初始化时返回
extern Vision_Recv_s recv;
extern fp32 INS_angle[3];
int16_t INS_angle_send[4];
uint8_t ins_buf[8];
extern float yaw_send;
static void ExchangInit();
static void Up_send_to_down();

void exchange_task()
{
	ExchangInit();
	for (;;)
	{
		osDelay(1);
		Up_send_to_down();
		VisionSend();
	}
}

static void ExchangInit()
{
	Vision_Init_Config_s vision_init_config = {
		.recv_config = {
			.header = VISION_RECV_HEADER,
		},
		.send_config = {
			.header = VISION_SEND_HEADER,
			.detect_color = VISION_DETECT_COLOR_RED,
			.reset_tracker = VISION_RESET_TRACKER_NO,
			.is_shoot = VISION_SHOOTING,
			.tail = VISION_SEND_TAIL,
		},
		.usart_config = {
			.recv_buff_size = VISION_RECV_SIZE,
			.usart_handle = &huart1,
		},

	};
	vision_recv_data = VisionInit(&vision_init_config);
}

//================================================上C向下C发送数据================================================//
static void Up_send_to_down()
{
	INS_angle_send[0] = INS_angle[0] * 100;
	INS_angle_send[1] = INS_angle[1] * 100;
	INS_angle_send[2] = INS_angle[2] * 100;
	INS_angle_send[3] = yaw_send * 100;


	ins_buf[0] = (INS_angle_send[0] >> 8) & 0xff;
	ins_buf[1] = INS_angle_send[1] >> 8;
	ins_buf[2] = (INS_angle_send[2] >> 8) & 0xff;
	ins_buf[3] = INS_angle_send[3] >> 8;
	ins_buf[4] = (INS_angle_send[4] >> 8) & 0xff;
	ins_buf[5] = INS_angle_send[5] >> 8;
	ins_buf[6] = (INS_angle_send[6] >> 8) & 0xff;
	ins_buf[7] = INS_angle_send[7] >> 8;
	can_remote(ins_buf, 0x55);
}