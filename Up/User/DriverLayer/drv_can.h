#ifndef __DRV_CAN_H
#define __DRV_CAN_H

#include "struct_typedef.h"
#include "chassis_task.h"
#include "Gimbal_task.h"
#include "Shoot_task.h"
#include "rc_potocal.h"

#define MOTOR_2006_0 0x200
#define MOTOR_2006_1 0x1ff
#define MOTOR_3508_0 0x200
#define MOTOR_3508_1 0x1ff
#define MOTOR_6020_0 0x1ff
#define MOTOR_6020_1 0x2ff

#define RC_ID_0 0x300
#define RC_ID_1 0x301

void CAN1_Init(void);
void CAN2_Init(void);
void can_receive(RC_ctrl_t *rc_ctrl, CAN_RxHeaderTypeDef rxHeader, uint8_t *rx_data); // 调用can来接收遥控器数据

void set_curruent(uint32_t motor_range, CAN_HandleTypeDef can_id, int16_t v1, int16_t v2, int16_t v3, int16_t v4);
void process_MotorInfo(motor_info_t *motor_info, uint8_t *rx_data);

#endif