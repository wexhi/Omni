
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "bsp_dwt.h"
#include "drv_can.h"
#include "drv_usart.h"
#include "bsp_init.h"
#include "robot_def.h"
#include "rc_potocal.h"
#include "arm_math.h"
#include "INS_task.h"
#include "exchange.h"
#include "Chassis_task.h"
#include "super_cap.h"
#include "Gimbal_task.h"
#include "stm32f4xx_it.h"

osThreadId Chassis_taskHandle;
osThreadId super_capHandle;
osThreadId UI_taskHandle;
osThreadId Gimbal_taskHandle;
osThreadId insTaskHandle;
osThreadId exchangeTaskHandle;

void OSTaskInit();
void StartINSTASK(void const *argument);

/**
 * @brief 机器人初始化,请在开启rtos之前调用.这也是唯一需要放入main函数的函数
 *
 */
void RobotInit()
{
    // 关闭中断,防止在初始化过程中发生中断
    // 请不要在初始化过程中使用中断和延时函数！
    // 若必须,则只允许使用DWT_Delay()
    __disable_irq();
    BSPInit();
    CAN1_Init();
    CAN2_Init();
    USART3_Init();

    Chassis_Init(); // 底盘初始化
    Gimbal_Init();  // 云台电机初始化
    ExchangInit();  // 交换机初始化
    OSTaskInit();
    __enable_irq();
}

void OSTaskInit()
{
    osThreadDef(instask, StartINSTASK, osPriorityRealtime, 0, 1024);
    insTaskHandle = osThreadCreate(osThread(instask), NULL); // 由于是阻塞读取传感器,为姿态解算设置较高优先级,确保�?1khz的频率执�?

    osThreadDef(Chassistask, Chassis_task, osPriorityNormal, 0, 512); // �����ƶ�����
    Chassis_taskHandle = osThreadCreate(osThread(Chassistask), NULL);

    osThreadDef(exchangeTask, exchange_task, osPriorityNormal, 0, 128);
    exchangeTaskHandle = osThreadCreate(osThread(exchangeTask), NULL);

    osThreadDef(GimbalTask, Gimbal_task, osPriorityNormal, 0, 512);
    Gimbal_taskHandle = osThreadCreate(osThread(GimbalTask), NULL);
}

__attribute__((noreturn)) void StartINSTASK(void const *argument)
{
    INS_Init(); // 确保BMI088被正确初始化.
    for (;;)
    {
        // 1kHz
        INS_Task();
        osDelay(1);
    }
}