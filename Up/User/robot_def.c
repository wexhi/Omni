#include "robot_def.h"
#include "ins_task.h"
#include "drv_can.h"
#include "drv_usart.h"
#include "bsp_delay.h"
#include "bsp_init.h"

/**
 * @brief 机器人初始化,请在开启rtos之前调用.这也是唯一需要放入main函数的函数
 *
 */
void RobotInit()
{
    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
    CAN1_Init();
    CAN2_Init();
    USART6_Init();
    USART3_Init();
    HAL_TIM_Base_Start(&htim1); 
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    BSPInit();
    INS_Init();
}