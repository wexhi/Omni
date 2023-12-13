#ifndef __DRV_CAN_H
#define __DRV_CAN_H

#include "struct_typedef.h"
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

void set_motor_current_chassis(uint8_t id_range, int16_t v1, int16_t v2, int16_t v3, int16_t v4);
void set_motor_current_gimbal(uint8_t id_range, int16_t v1, int16_t v2, int16_t v3, int16_t v4);
void set_motor_current_gimbal2(uint8_t id_range, int16_t v1, int16_t v2, int16_t v3, int16_t v4);
void set_motor_current_shoot(uint8_t id_range, int16_t v1, int16_t v2, int16_t v3, int16_t v4);
void set_curruent(uint32_t motor_range, CAN_HandleTypeDef can_id, int16_t v1, int16_t v2, int16_t v3, int16_t v4);
void can_remote(uint8_t sbus_buf[], uint8_t can_send_id);
void process_MotorInfo(motor_info_t *motor_info, uint8_t *rx_data);

#endif