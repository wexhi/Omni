#include "struct_typedef.h"
#include "exchange.h"
#include "INS_task.h"
#include "drv_can.h"
#include <string.h>
#include "miniPC_process.h"

static attitude_t *attitude_data; // 姿态数据指针
static float pitch_angle = 0;	  // 云台pitch轴角度
int16_t INS_angle_send[4];
uint8_t ins_buf[8];

extern float yaw_send;
extern uint8_t is_tracking;
extern Vision_Recv_s recv;

static void ExchangInit();		  // 上下位机通信初始化
static void VisionAngleProcess(); // 视觉角度处理
static void Up_send_to_down();	  // 上C向下C发送数据

void exchange_task()
{
	ExchangInit();
	for (;;)
	{
		Up_send_to_down();
		VisionAngleProcess();
		VisionSetAltitude(attitude_data->Yaw, pitch_angle, attitude_data->Roll);
		VisionSend();
		osDelay(1);
	}
}

/**
 * @brief 上下位机通信初始化
 *
 */
static void ExchangInit()
{
	attitude_data = INS_Init();

	Vision_Init_Config_s vision_init_config = {
		.recv_config = {
			.header = VISION_RECV_HEADER,
		},
		.send_config = {
			.header = VISION_SEND_HEADER,
			.detect_color = VISION_DETECT_COLOR_BLUE,
			.reset_tracker = VISION_RESET_TRACKER_NO,
			.is_shoot = VISION_SHOOTING,
			.tail = VISION_SEND_TAIL,
		},
		.usart_config = {
			.recv_buff_size = VISION_RECV_SIZE,
			.usart_handle = &huart1,
		},

	};
	VisionInit(&vision_init_config);
}

/**
 * @brief 视觉角度处理
 *
 */
static void VisionAngleProcess()
{
	pitch_angle = attitude_data->Pitch;
	if (pitch_angle > -180 && pitch_angle < -140)
		pitch_angle = -(pitch_angle + 180);
	else if (pitch_angle > 140 && pitch_angle < 180)
		pitch_angle = -(pitch_angle - 180);
	else
		pitch_angle = 0.0f;
}

//================================================上C向下C发送数据================================================//
static void Up_send_to_down()
{
	INS_angle_send[0] = attitude_data->Yaw * 100;
	INS_angle_send[1] = (int16_t)(attitude_data->Gyro[2] * 2000);
	INS_angle_send[3] = yaw_send * 100;

	ins_buf[0] = (INS_angle_send[0] >> 8) & 0xff;
	ins_buf[1] = INS_angle_send[0] >> 8;
	ins_buf[2] = (INS_angle_send[1] >> 8) & 0xff;
	ins_buf[3] = INS_angle_send[1] >> 8;
	// ins_buf[4] = (INS_angle_send[4] >> 8) & 0xff;
	// ins_buf[5] = INS_angle_send[5] >> 8;
	ins_buf[4] = is_tracking;
	ins_buf[5] = is_tracking;
	ins_buf[6] = (INS_angle_send[3] >> 8) & 0xff;
	ins_buf[7] = INS_angle_send[3] >> 8;
	can_remote(ins_buf, 0x55);
}