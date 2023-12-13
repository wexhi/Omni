/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       INS_task.c/h
  * @brief      use bmi088 to calculate the euler angle. no use ist8310, so only
  *             enable data ready pin to save cpu time.enalbe bmi088 data ready
  *             enable spi DMA to save the time spi transmit
  *             Ö÷ÒªÀûÓÃÍÓÂÝÒÇbmi088£¬´ÅÁ¦¼Æist8310£¬Íê³É×ËÌ¬½âËã£¬µÃ³öÅ·À­½Ç£¬
  *             Ìá¹©Í¨¹ýbmi088µÄdata ready ÖÐ¶ÏÍê³ÉÍâ²¿´¥·¢£¬¼õÉÙÊý¾ÝµÈ´ýÑÓ³Ù
  *             Í¨¹ýDMAµÄSPI´«Êä½ÚÔ¼CPUÊ±¼ä.
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V2.0.0     Nov-11-2019     RM              1. support bmi088, but don't support mpu6500
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "INS_task.h"
#include "main.h"
#include "arm_math.h"
#include "cmsis_os.h"

#include "bsp_imu_pwm.h"
#include "bsp_spi.h"
#include "bmi088driver.h"

#include "pid_imu.h"

#define IMU_temp_PWM(pwm) imu_pwm_set(pwm) // pwm¸ø¶¨

#define BMI088_BOARD_INSTALL_SPIN_MATRIX \
    {1.0f, 0.0f, 0.0f},                  \
        {0.0f, 1.0f, 0.0f},              \
    {                                    \
        0.0f, 0.0f, 1.0f                 \
    }

/**
 * @brief          rotate the gyro, accel and mag, and calculate the zero drift, because sensors have
 *                 different install derection.
 * @param[out]     gyro: after plus zero drift and rotate
 * @param[out]     accel: after plus zero drift and rotate
 * @param[out]     mag: after plus zero drift and rotate
 * @param[in]      bmi088: gyro and accel data
 * @param[in]      ist8310: mag data
 * @retval         none
 */
/**
 * @brief          Ðý×ªÍÓÂÝÒÇ,¼ÓËÙ¶È¼ÆºÍ´ÅÁ¦¼Æ,²¢¼ÆËãÁãÆ¯,ÒòÎªÉè±¸ÓÐ²»Í¬°²×°·½Ê½
 * @param[out]     gyro: ¼ÓÉÏÁãÆ¯ºÍÐý×ª
 * @param[out]     accel: ¼ÓÉÏÁãÆ¯ºÍÐý×ª
 * @param[out]     mag: ¼ÓÉÏÁãÆ¯ºÍÐý×ª
 * @param[in]      bmi088: ÍÓÂÝÒÇºÍ¼ÓËÙ¶È¼ÆÊý¾Ý
 * @param[in]      ist8310: ´ÅÁ¦¼ÆÊý¾Ý
 * @retval         none
 */
static void imu_cali_slove(fp32 gyro[3], fp32 accel[3], bmi088_real_data_t *bmi088);

/**
 * @brief          control the temperature of bmi088
 * @param[in]      temp: the temperature of bmi088
 * @retval         none
 */
/**
 * @brief          ¿ØÖÆbmi088µÄÎÂ¶È
 * @param[in]      temp:bmi088µÄÎÂ¶È
 * @retval         none
 */
static void imu_temp_control(fp32 temp);

/**
 * @brief          open the SPI DMA accord to the value of imu_update_flag
 * @param[in]      none
 * @retval         none
 */
/**
 * @brief          ¸ù¾Ýimu_update_flagµÄÖµ¿ªÆôSPI DMA
 * @param[in]      temp:bmi088µÄÎÂ¶È
 * @retval         none
 */
static void imu_cmd_spi_dma(void);

extern SPI_HandleTypeDef hspi1;

static TaskHandle_t INS_task_local_handler;

uint8_t gyro_dma_rx_buf[SPI_DMA_GYRO_LENGHT];
uint8_t gyro_dma_tx_buf[SPI_DMA_GYRO_LENGHT] = {0x82, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

uint8_t accel_dma_rx_buf[SPI_DMA_ACCEL_LENGHT];
uint8_t accel_dma_tx_buf[SPI_DMA_ACCEL_LENGHT] = {0x92, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

uint8_t accel_temp_dma_rx_buf[SPI_DMA_ACCEL_TEMP_LENGHT];
uint8_t accel_temp_dma_tx_buf[SPI_DMA_ACCEL_TEMP_LENGHT] = {0xA2, 0xFF, 0xFF, 0xFF};

volatile uint8_t gyro_update_flag = 0;
volatile uint8_t accel_update_flag = 0;
volatile uint8_t accel_temp_update_flag = 0;
volatile uint8_t imu_start_dma_flag = 0;

bmi088_real_data_t bmi088_real_data;
fp32 gyro_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};

fp32 accel_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
fp32 accel_offset[3];

static uint8_t first_temperate;
static const fp32 imu_temp_PID[3] = {TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD};
pid_type_def imu_temp_pid;

static const float timing_time = 0.001f; // tast run time , unit s.ÈÎÎñÔËÐÐµÄÊ±¼ä µ¥Î» s

// ¼ÓËÙ¶È¼ÆµÍÍ¨ÂË²¨
static fp32 accel_fliter_1[3] = {0.0f, 0.0f, 0.0f};
static fp32 accel_fliter_2[3] = {0.0f, 0.0f, 0.0f};
static fp32 accel_fliter_3[3] = {0.0f, 0.0f, 0.0f};
static const fp32 fliter_num[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};

fp32 gyrooffset[3];
fp32 INS_gyro[3] = {0.0f, 0.0f, 0.0f};
fp32 INS_accel[3] = {0.0f, 0.0f, 0.0f};
fp32 INS_mag[3] = {0.0f, 0.0f, 0.0f};
fp32 INS_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f};
fp32 INS_angle[3] = {0.0f, 0.0f, 0.0f};

volatile float twoKp = 1.0;                                                // 2 * proportional gain (Kp)
volatile float twoKi = 0;                                                  // 2 * integral gain (Ki)
volatile float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f; // integral error terms scaled by Ki
#define sampleFreq 1000.0f

void AHRS_init(fp32 quat[4], const fp32 accel[3], const fp32 mag[3])
{
    quat[0] = 1.0f;
    quat[1] = 0.0f;
    quat[2] = 0.0f;
    quat[3] = 0.0f;
}
fp32 invSqrt(fp32 num)
{
    fp32 halfnum = 0.5f * num;
    fp32 y = num;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(fp32 *)&i;
    y = y * (1.5f - (halfnum * y * y));
    return y;
}
void MahonyAHRSupdateIMU(float q[4], float gx, float gy, float gz, float ax, float ay, float az)
{

    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {

        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Estimated direction of gravity and vector perpendicular to magnetic flux
        halfvx = q[1] * q[3] - q[0] * q[2];
        halfvy = q[0] * q[1] + q[2] * q[3];
        halfvz = q[0] * q[0] - 0.5f + q[3] * q[3];

        // Error is sum of cross product between estimated and measured direction of gravity
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // Compute and apply integral feedback if enabled
        if (twoKi > 0.0f)
        {
            integralFBx += twoKi * halfex * (1.0f / sampleFreq); // integral error scaled by Ki
            integralFBy += twoKi * halfey * (1.0f / sampleFreq);
            integralFBz += twoKi * halfez * (1.0f / sampleFreq);
            gx += integralFBx; // apply integral feedback
            gy += integralFBy;
            gz += integralFBz;
        }
        else
        {
            integralFBx = 0.0f; // prevent integral windup
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        // Apply proportional feedback
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    // Integrate rate of change of quaternion
    gx *= (0.5f * (1.0f / sampleFreq)); // pre-multiply common factors
    gy *= (0.5f * (1.0f / sampleFreq));
    gz *= (0.5f * (1.0f / sampleFreq));
    qa = q[0];
    qb = q[1];
    qc = q[2];
    q[0] += (-qb * gx - qc * gy - q[3] * gz);
    q[1] += (qa * gx + qc * gz - q[3] * gy);
    q[2] += (qa * gy - qb * gz + q[3] * gx);
    q[3] += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    recipNorm = invSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    q[0] *= recipNorm;
    q[1] *= recipNorm;
    q[2] *= recipNorm;
    q[3] *= recipNorm;
}

void get_angle(const fp32 q[4], fp32 *yaw, fp32 *pitch, fp32 *roll)
{

    *yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f) * 57.3;
    *pitch = asinf(-2.0f * (q[1] * q[3] - q[0] * q[2])) * 57.3;
    *roll = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f) * 57.3;
}

void INS_gyrooffset_calculate()
{
    int cnt = 0;
    while (cnt++ < 20000)
    {
        BMI088_read(bmi088_real_data.gyro, bmi088_real_data.accel, &bmi088_real_data.temp);
        gyrooffset[0] -= bmi088_real_data.gyro[0] * 0.00005;
        gyrooffset[1] -= bmi088_real_data.gyro[1] * 0.00005;
        gyrooffset[2] -= bmi088_real_data.gyro[2] * 0.00005;
    }
}

/**
 * @brief          imuÈÎÎñ, ³õÊ¼»¯ bmi088, ist8310, ¼ÆËãÅ·À­½Ç
 * @param[in]      pvParameters: NULL
 * @retval         none
 */

void INS_task(void const *pvParameters)
{
    // wait a time

    osDelay(INS_TASK_INIT_TIME);
    while (BMI088_init())
    {
        osDelay(100);
    }

    BMI088_read(bmi088_real_data.gyro, bmi088_real_data.accel, &bmi088_real_data.temp);
    INS_gyrooffset_calculate();
    // rotate and zero drift
    imu_cali_slove(INS_gyro, INS_accel, &bmi088_real_data);

    PID_init(&imu_temp_pid, PID_POSITION, imu_temp_PID, TEMPERATURE_PID_MAX_OUT, TEMPERATURE_PID_MAX_IOUT);
    AHRS_init(INS_quat, INS_accel, INS_mag);

    accel_fliter_1[0] = accel_fliter_2[0] = accel_fliter_3[0] = INS_accel[0];
    accel_fliter_1[1] = accel_fliter_2[1] = accel_fliter_3[1] = INS_accel[1];
    accel_fliter_1[2] = accel_fliter_2[2] = accel_fliter_3[2] = INS_accel[2];
    // get the handle of task
    // »ñÈ¡µ±Ç°ÈÎÎñµÄÈÎÎñ¾ä±ú£¬
    INS_task_local_handler = xTaskGetHandle(pcTaskGetName(NULL));

    // set spi frequency
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;

    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
    }

    SPI1_DMA_init((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);

    imu_start_dma_flag = 1;

    while (1)
    {
        // wait spi DMA tansmit done
        // µÈ´ýSPI DMA´«Êä
        while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS)
        {
        }

        if (gyro_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            gyro_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            BMI088_gyro_read_over(gyro_dma_rx_buf + BMI088_GYRO_RX_BUF_DATA_OFFSET, bmi088_real_data.gyro);
        }

        if (accel_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            accel_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            BMI088_accel_read_over(accel_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, bmi088_real_data.accel, &bmi088_real_data.time);
        }

        if (accel_temp_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            accel_temp_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            BMI088_temperature_read_over(accel_temp_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, &bmi088_real_data.temp);
            imu_temp_control(bmi088_real_data.temp);
        }

        // rotate and zero drift
        imu_cali_slove(INS_gyro, INS_accel, &bmi088_real_data);

        // ¼ÓËÙ¶È¼ÆµÍÍ¨ÂË²¨
        // accel low-pass filter
        accel_fliter_1[0] = accel_fliter_2[0];
        accel_fliter_2[0] = accel_fliter_3[0];

        accel_fliter_3[0] = accel_fliter_2[0] * fliter_num[0] + accel_fliter_1[0] * fliter_num[1] + INS_accel[0] * fliter_num[2];

        accel_fliter_1[1] = accel_fliter_2[1];
        accel_fliter_2[1] = accel_fliter_3[1];

        accel_fliter_3[1] = accel_fliter_2[1] * fliter_num[0] + accel_fliter_1[1] * fliter_num[1] + INS_accel[1] * fliter_num[2];

        accel_fliter_1[2] = accel_fliter_2[2];
        accel_fliter_2[2] = accel_fliter_3[2];

        accel_fliter_3[2] = accel_fliter_2[2] * fliter_num[0] + accel_fliter_1[2] * fliter_num[1] + INS_accel[2] * fliter_num[2];

        MahonyAHRSupdateIMU(INS_quat, INS_gyro[0], INS_gyro[1], INS_gyro[2], accel_fliter_3[0], accel_fliter_3[1], accel_fliter_3[2]);
        get_angle(INS_quat, INS_angle + INS_YAW_ADDRESS_OFFSET, INS_angle + INS_PITCH_ADDRESS_OFFSET, INS_angle + INS_ROLL_ADDRESS_OFFSET);
    }
}

/**
 * @brief          Ðý×ªÍÓÂÝÒÇ,¼ÓËÙ¶È¼ÆºÍ´ÅÁ¦¼Æ,²¢¼ÆËãÁãÆ¯,ÒòÎªÉè±¸ÓÐ²»Í¬°²×°·½Ê½
 * @param[out]     gyro: ¼ÓÉÏÁãÆ¯ºÍÐý×ª
 * @param[out]     accel: ¼ÓÉÏÁãÆ¯ºÍÐý×ª
 * @param[out]     mag: ¼ÓÉÏÁãÆ¯ºÍÐý×ª
 * @param[in]      bmi088: ÍÓÂÝÒÇºÍ¼ÓËÙ¶È¼ÆÊý¾Ý
 * @param[in]      ist8310: ´ÅÁ¦¼ÆÊý¾Ý
 * @retval         none
 */
static void imu_cali_slove(fp32 gyro[3], fp32 accel[3], bmi088_real_data_t *bmi088)
{
    for (uint8_t i = 0; i < 3; i++)
    {
        gyro[i] = bmi088->gyro[0] * gyro_scale_factor[i][0] + bmi088->gyro[1] * gyro_scale_factor[i][1] + bmi088->gyro[2] * gyro_scale_factor[i][2] + gyrooffset[i];
        accel[i] = bmi088->accel[0] * accel_scale_factor[i][0] + bmi088->accel[1] * accel_scale_factor[i][1] + bmi088->accel[2] * accel_scale_factor[i][2] + accel_offset[i];
    }
}

/**
 * @brief          ¿ØÖÆbmi088µÄÎÂ¶È
 * @param[in]      temp:bmi088µÄÎÂ¶È
 * @retval         none
 */
static void imu_temp_control(fp32 temp)
{
    uint16_t tempPWM;
    static uint8_t temp_constant_time = 0;
    if (first_temperate)
    {
        PID_calc(&imu_temp_pid, temp, 40);
        if (imu_temp_pid.out < 0.0f)
        {
            imu_temp_pid.out = 0.0f;
        }
        tempPWM = (uint16_t)imu_temp_pid.out;
        IMU_temp_PWM(tempPWM);
    }
    else
    {
        // ÔÚÃ»ÓÐ´ïµ½ÉèÖÃµÄÎÂ¶È£¬Ò»Ö±×î´ó¹¦ÂÊ¼ÓÈÈ
        // in beginning, max power
        if (temp > 40)
        {
            temp_constant_time++;
            if (temp_constant_time > 200)
            {
                // ´ïµ½ÉèÖÃÎÂ¶È£¬½«»ý·ÖÏîÉèÖÃÎªÒ»°ë×î´ó¹¦ÂÊ£¬¼ÓËÙÊÕÁ²
                //
                first_temperate = 1;
                imu_temp_pid.Iout = MPU6500_TEMP_PWM_MAX / 2.0f;
            }
        }

        IMU_temp_PWM(MPU6500_TEMP_PWM_MAX - 1);
    }
}

/**
 * @brief          »ñÈ¡Å·À­½Ç, 0:yaw, 1:pitch, 2:roll µ¥Î» rad
 * @param[in]      none
 * @retval         INS_angleµÄÖ¸Õë
 */
const fp32 *get_INS_angle_point(void)
{
    return INS_angle;
}

/**
 * @brief          »ñÈ¡½ÇËÙ¶È,0:xÖá, 1:yÖá, 2:rollÖá µ¥Î» rad/s
 * @param[in]      none
 * @retval         INS_gyroµÄÖ¸Õë
 */
extern const fp32 *get_gyro_data_point(void)
{
    return INS_gyro;
}

/**
 * @brief          »ñÈ¡¼ÓËÙ¶È,0:xÖá, 1:yÖá, 2:rollÖá µ¥Î» m/s2
 * @param[in]      none
 * @retval         INS_accelµÄÖ¸Õë
 */
extern const fp32 *get_accel_data_point(void)
{
    return INS_accel;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == INT1_ACCEL_Pin)
    {
        // detect_hook(BOARD_ACCEL_TOE);
        accel_update_flag |= 1 << IMU_DR_SHFITS;
        accel_temp_update_flag |= 1 << IMU_DR_SHFITS;
        if (imu_start_dma_flag)
        {
            imu_cmd_spi_dma();
        }
    }
    else if (GPIO_Pin == INT1_GYRO_Pin)
    {
        // detect_hook(BOARD_GYRO_TOE);
        gyro_update_flag |= 1 << IMU_DR_SHFITS;
        if (imu_start_dma_flag)
        {
            imu_cmd_spi_dma();
        }
    }

    else if (GPIO_Pin == GPIO_PIN_0)
    {

        // wake up the task
        // »½ÐÑÈÎÎñ
        if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
        {
            static BaseType_t xHigherPriorityTaskWoken;
            vTaskNotifyGiveFromISR(INS_task_local_handler, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}

/**
 * @brief          ¸ù¾Ýimu_update_flagµÄÖµ¿ªÆôSPI DMA
 * @param[in]      temp:bmi088µÄÎÂ¶È
 * @retval         none
 */
static void imu_cmd_spi_dma(void)
{
    // ¿ªÆôÍÓÂÝÒÇµÄDMA´«Êä
    if ((gyro_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN) && !(accel_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
    {
        gyro_update_flag &= ~(1 << IMU_DR_SHFITS);
        gyro_update_flag |= (1 << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);
        return;
    }
    // ¿ªÆô¼ÓËÙ¶È¼ÆµÄDMA´«Êä
    if ((accel_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN) && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
    {
        accel_update_flag &= ~(1 << IMU_DR_SHFITS);
        accel_update_flag |= (1 << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)accel_dma_tx_buf, (uint32_t)accel_dma_rx_buf, SPI_DMA_ACCEL_LENGHT);
        return;
    }

    if ((accel_temp_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN) && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_update_flag & (1 << IMU_SPI_SHFITS)))
    {
        accel_temp_update_flag &= ~(1 << IMU_DR_SHFITS);
        accel_temp_update_flag |= (1 << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)accel_temp_dma_tx_buf, (uint32_t)accel_temp_dma_rx_buf, SPI_DMA_ACCEL_TEMP_LENGHT);
        return;
    }
}

void DMA2_Stream2_IRQHandler(void)
{

    if (DMA2->LISR & (1 << 21))
    {
        DMA2->LIFCR = (uint32_t)((DMA_LISR_FEIF0 | DMA_LISR_DMEIF0 | DMA_LISR_TEIF0 | DMA_LISR_HTIF0 | DMA_LISR_TCIF0) << 21);

        // gyro read over
        // ÍÓÂÝÒÇ¶ÁÈ¡Íê±Ï
        if (gyro_update_flag & (1 << IMU_SPI_SHFITS))
        {
            gyro_update_flag &= ~(1 << IMU_SPI_SHFITS);
            gyro_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
        }

        // accel read over
        // ¼ÓËÙ¶È¼Æ¶ÁÈ¡Íê±Ï
        if (accel_update_flag & (1 << IMU_SPI_SHFITS))
        {
            accel_update_flag &= ~(1 << IMU_SPI_SHFITS);
            accel_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        }
        // temperature read over
        // ÎÂ¶È¶ÁÈ¡Íê±Ï
        if (accel_temp_update_flag & (1 << IMU_SPI_SHFITS))
        {
            accel_temp_update_flag &= ~(1 << IMU_SPI_SHFITS);
            accel_temp_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        }

        imu_cmd_spi_dma();

        if (gyro_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            __HAL_GPIO_EXTI_GENERATE_SWIT(GPIO_PIN_0);
        }
    }
}
