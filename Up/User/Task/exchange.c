#include "struct_typedef.h"
#include "exchange.h"
#include "ins_task.h"
#include "drv_can.h"
extern INS_t INS;

ins_data_t ins_data;
int16_t yaw_to_down = 0;
int16_t pitch_to_down = 0;
int16_t roll_to_down = 0;
int16_t yaw_total_angle_to_down = 0;

static void Up_send_to_down();

void exchange_task()
{
	for (;;)
	{
		osDelay(1);
		yaw_to_down = INS.Yaw;
		pitch_to_down = INS.Pitch;
		roll_to_down = INS.Roll;
		yaw_total_angle_to_down = INS.YawTotalAngle;
		Up_send_to_down();
	}
}

//================================================上C向下C发送数据================================================//
static void Up_send_to_down()
{
	uint32_t send_mail_box;
	CAN_TxHeaderTypeDef tx_header;
	uint8_t test[2];
	tx_header.StdId = 0x55;
	tx_header.IDE = CAN_ID_STD;
	tx_header.RTR = CAN_RTR_DATA;
	tx_header.DLC = 0x02;
	test[0] = ((int16_t)INS.Yaw >> 8) & 0xFF;
	test[1] = (int16_t)INS.Yaw & 0xFF;

	HAL_CAN_AddTxMessage(&hcan2, &tx_header, test, &send_mail_box);
}