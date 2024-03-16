#include "cmsis_os.h"

typedef struct
{
    fp32 yaw;
    fp32 yaw_gyro;
} UP_C_angle_t;

void exchange_task();
void ExchangInit();