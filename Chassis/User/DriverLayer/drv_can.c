#include "drv_can.h"
#define GIMBAL_YAW_ID 0x20b
#define CHASSIS_ID_START 0x201
#define CHASSIS_ID_END 0x204
#define SHOOTER_ID_START 0x205
#define SHOOTER_ID_END 0x208
#define GIMBAL_PITCH_ID 0x209
#define POWERDATA_ID 0x211

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern RC_ctrl_t rc_ctrl;
uint16_t can_cnt_1 = 0;

extern gimbal_t gimbal_Yaw, gimbal_Pitch;
extern chassis_t chassis;
extern shooter_t shooter;

float powerdata[4];
uint16_t pPowerdata[8];

uint16_t setpower = 5500;
int canerror = 0;

void CAN1_Init(void)
{
  CAN_FilterTypeDef can_filter;

  can_filter.FilterBank = 0;                      // filter 0
  can_filter.FilterMode = CAN_FILTERMODE_IDMASK;  // 标识符屏蔽位模式
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT; // 过滤器位宽为单个32位
  can_filter.FilterIdHigh = 0;                    // 标识符寄存器
  can_filter.FilterIdLow = 0;                     // 标识符寄存器
  can_filter.FilterMaskIdHigh = 0;                // 屏蔽寄存器
  can_filter.FilterMaskIdLow = 0;                 // 屏蔽寄存器   set mask 0 to receive all can id
  can_filter.FilterFIFOAssignment = CAN_RX_FIFO0; // assign to fifo0，接收器是FIFO0
  can_filter.FilterActivation = ENABLE;           // enable can filter
  can_filter.SlaveStartFilterBank = 14;

  HAL_CAN_ConfigFilter(&hcan1, &can_filter);                         // init can filter
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); // 使能can的FIFO0中断
  HAL_CAN_Start(&hcan1);                                             // 启动can1
}

void CAN2_Init(void)
{
  CAN_FilterTypeDef can_filter;

  can_filter.FilterBank = 14;                     // filter 14
  can_filter.FilterMode = CAN_FILTERMODE_IDMASK;  // 标识符屏蔽位模式
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT; // 过滤器位宽为单个32位
  can_filter.FilterIdHigh = 0;                    // 标识符寄存器
  can_filter.FilterIdLow = 0;                     // 标识符寄存器
  can_filter.FilterMaskIdHigh = 0;                // 屏蔽寄存器
  can_filter.FilterMaskIdLow = 0;                 // 屏蔽寄存器   set mask 0 to receive all can id
  can_filter.FilterFIFOAssignment = CAN_RX_FIFO0; // assign to fifo0，接收器是FIFO0
  can_filter.FilterActivation = ENABLE;           // enable can filter
  can_filter.SlaveStartFilterBank = 14;

  HAL_CAN_ConfigFilter(&hcan2, &can_filter); // init can filter
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_Start(&hcan2); // 启动can2
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) // 接受中断回调函数
{
  CAN_RxHeaderTypeDef rx_header;

  if (hcan->Instance == CAN1)
  {
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data); // receive can1 data

    // 云台电机信息接收
    if (rx_header.StdId == GIMBAL_YAW_ID)
    {
      process_MotorInfo(&gimbal_Yaw.motor_info, rx_data);
    }
    // 底盤电机信息接收
    if (rx_header.StdId >= CHASSIS_ID_START && rx_header.StdId <= CHASSIS_ID_END)
    {
      process_MotorInfo(&chassis.motor_info[rx_header.StdId - CHASSIS_ID_START], rx_data);
    }
  }

  if (hcan->Instance == CAN2)
  {
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data); // receive can2 data
    // 發射機構电机信息接收
    if (rx_header.StdId >= SHOOTER_ID_START && rx_header.StdId <= SHOOTER_ID_END)
    {
      process_MotorInfo(&shooter.motor_info[rx_header.StdId - SHOOTER_ID_START], rx_data);
    }
    // 云台电机信息接收
    if (rx_header.StdId == GIMBAL_PITCH_ID)
    {
      process_MotorInfo(&gimbal_Pitch.motor_info, rx_data);
    }

    if (rx_header.StdId == 0x211)
    {

      extern float powerdata[4];
      uint16_t *pPowerdata = (uint16_t *)rx_data;

      powerdata[0] = (float)pPowerdata[0] / 100.f; // 输入电压
      powerdata[1] = (float)pPowerdata[1] / 100.f; // 电容电压
      powerdata[2] = (float)pPowerdata[2] / 100.f; // 输入电流
      powerdata[3] = (float)pPowerdata[3] / 100.f; // P
    }
  }
}

void can_remote(uint8_t sbus_buf[], uint8_t can_send_id) // 调用can来发送遥控器数据
{
  // CAN_TxHeaderTypeDef tx_header;

  // tx_header.StdId = can_send_id; // 如果id_range==0则等于0x1ff,id_range==1则等于0x2ff（ID号）
  // tx_header.IDE = CAN_ID_STD;    // 标准帧
  // tx_header.RTR = CAN_RTR_DATA;  // 数据帧
  // tx_header.DLC = 8;             // 发送数据长度（字节）

  //  HAL_CAN_AddTxMessage(&hcan1, &tx_header, sbus_buf, (uint32_t *)CAN_TX_MAILBOX0);
}

// 一步解决所有电机控制
void set_curruent(uint32_t motor_range, CAN_HandleTypeDef can_id, int16_t v1, int16_t v2, int16_t v3, int16_t v4)
{
  CAN_TxHeaderTypeDef tx_header;
  uint8_t tx_data[8];

  tx_header.StdId = motor_range; // 控制电机的ID号
  tx_header.IDE = CAN_ID_STD;    // 标准帧
  tx_header.RTR = CAN_RTR_DATA;  // 数据帧

  tx_header.DLC = 8; // 发送数据长度（字节）

  tx_data[0] = (v1 >> 8) & 0xff; // 先发高八位
  tx_data[1] = (v1) & 0xff;
  tx_data[2] = (v2 >> 8) & 0xff;
  tx_data[3] = (v2) & 0xff;
  tx_data[4] = (v3 >> 8) & 0xff;
  tx_data[5] = (v3) & 0xff;
  tx_data[6] = (v4 >> 8) & 0xff;
  tx_data[7] = (v4) & 0xff;
  HAL_CAN_AddTxMessage(&can_id, &tx_header, tx_data, (uint32_t *)CAN_TX_MAILBOX0);
}

// 接收代码
void process_MotorInfo(motor_info_t *motor_info, uint8_t *rx_data)
{
  motor_info->rotor_angle = ((rx_data[0] << 8) | rx_data[1]);
  motor_info->rotor_speed = ((rx_data[2] << 8) | rx_data[3]);
  motor_info->torque_current = ((rx_data[4] << 8) | rx_data[5]);
  motor_info->temp = rx_data[6];
}