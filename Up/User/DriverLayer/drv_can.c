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

uint16_t rc_flag = 0;
uint16_t rc_tmp[5];
int16_t mouse_x;
// extren uint16_t w_flag;
// extren uint16_t s_flag;
// extren uint16_t a_flag;
// extren uint16_t d_flag;
// extren uint16_t q_flag;
// extren uint16_t e_flag;
// extren uint16_t shift_flag;
// extren uint16_t ctrl_flag;
// extren uint8_t press_left;
// extren uint8_t press_right;
// extren uint16_t r_flag;
// extren uint16_t f_flag;
// extren uint16_t g_flag;
// extren uint16_t z_flag;
// extren uint16_t x_flag;
// extren uint16_t c_flag;
// extren uint16_t v_flag;
// extren uint16_t b_flag;

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

    // 上C板收遥控器数据
    can_receive(&rc_ctrl, rx_header, rx_data);

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

void can_receive(RC_ctrl_t *rc_ctrl, CAN_RxHeaderTypeDef rxHeader, uint8_t *rx_data) // 调用can来接收遥控器数据
{
  if (rxHeader.StdId == 0x300)
  {
    rc_tmp[0] = (rx_data[0] << 8) | rx_data[1];
    rc_tmp[1] = (rx_data[2] << 8) | rx_data[3];
    rc_tmp[2] = (rx_data[4] << 8) | rx_data[5];
    rc_tmp[3] = (rx_data[6] << 8) | rx_data[7];
    rc_flag = 1;
  }
  else if (rxHeader.StdId == 0x301)
  {
    rc_ctrl->rc.ch[0] = rc_tmp[0];
    rc_ctrl->rc.ch[1] = rc_tmp[1];
    rc_ctrl->rc.ch[2] = rc_tmp[2];
    rc_ctrl->rc.ch[3] = rc_tmp[3];
    rc_ctrl->rc.ch[4] = (rx_data[0] << 8) | rx_data[1];
    rc_ctrl->rc.s[0] = rx_data[2];
    rc_ctrl->rc.s[1] = rx_data[3];

    w_flag = (rx_data[5] & 0x01);
    s_flag = (rx_data[5] & 0x02);
    a_flag = (rx_data[5] & 0x04);
    d_flag = (rx_data[5] & 0x08);
    q_flag = (rx_data[5] & 0x40);
    e_flag = (rx_data[5] & 0x80);
    shift_flag = (rx_data[5] & 0x10);
    ctrl_flag = (rx_data[5] & 0x20);

    r_flag = (rx_data[4] & 0x01);
    f_flag = (rx_data[4] & 0x02);
    g_flag = (rx_data[4] & 0x04);
    z_flag = (rx_data[4] & 0x08);
    x_flag = (rx_data[4] & 0x10);
    c_flag = (rx_data[4] & 0x20);
    v_flag = (rx_data[4] & 0x40);
    b_flag = (rx_data[4] & 0x80);

    mouse_x = (rx_data[6] << 8) | rx_data[7];

    rc_flag = 0;
  }
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