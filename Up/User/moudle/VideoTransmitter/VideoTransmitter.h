/**
 * @file VideoTransmitter.h
 * @author wexhi (wexhi@qq.com)
 * @brief  用于图传链路的接收以及解析
 * @version 0.1
 * @date 2024-03-27
 * @todo 图传链路应该属于遥控器控制的附庸，后续考虑合并或转移
 *
 * @copyright Copyright (c) 2024 CQU QianLi EC 2024 all rights reserved
 *
 */

#ifndef VIDEOTRANSMITTER_H
#define VIDEOTRANSMITTER_H

#include "stdint.h"
#include "main.h"
#include "usart.h"

#include "remote_control.h"
#include "referee_protocol.h"

#pragma pack(1)

typedef struct
{

    xFrameHeader FrameHeader; // 接收到的帧头信息
    uint16_t CmdID;           // 命令码
    custom_robot_data_t custom_data; // 自定义数据
    remote_control_t key_data; // 遥控器数据

    Key_t key[3]; // 改为位域后的键盘索引,空间减少8倍,速度增加16~倍

    uint8_t key_count[3][16];
} Video_ctrl_t;

#pragma pack()

Video_ctrl_t *VideoTransmitterControlInit(UART_HandleTypeDef *video_usart_handle);

#endif // !VIDEOTRANSMITTER_H