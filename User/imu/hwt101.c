#include "hwt101.h"
#include "usart.h"
#include "raspberry.h"

#define HWT101_FRAME_HEADER       0x55
#define HWT101_GYRO_FRAME_TYPE    0x52
#define HWT101_ANGLE_FRAME_TYPE   0x53
#define HWT101_FRAME_LENGTH       11

volatile uint32_t hwt_rx_count = 0;
volatile uint32_t hwt_frame_count = 0;
volatile uint32_t hwt_angle_count = 0;
volatile uint32_t hwt_gyro_count = 0;
volatile uint32_t hwt_last_rx_tick = 0;
static volatile float imu_yaw;
static volatile float imu_gyro_z;
volatile uint32_t imu_gyro_last_tick = 0;
static volatile float imu_zero_yaw;
static volatile uint8_t imu_yaw_valid;
volatile uint32_t hwt_checksum_error_count = 0;
volatile uint32_t hwt_rx_restart_error_count = 0;

static uint8_t hwt_rx_byte;
static uint8_t hwt_frame[HWT101_FRAME_LENGTH];
static uint8_t hwt_frame_index;

static float HWT101_WrapAngle(float angle)
{
  while (angle > 180)
  {
    angle -= 360;
  }
  while (angle < -180)
  {
    angle += 360;
  }
  return angle;
}

/* 校验失败后寻找帧内的下一个 0x55，以便尽快恢复同步。 */
static void HWT101_Resync(void)
{
  uint8_t start;
  uint8_t i;

  for (start = 1; start < HWT101_FRAME_LENGTH; start++)
  {
    if (hwt_frame[start] == HWT101_FRAME_HEADER)
    {
      hwt_frame_index = HWT101_FRAME_LENGTH - start;
      for (i = 0; i < hwt_frame_index; i++)
      {
        hwt_frame[i] = hwt_frame[start + i];
      }
      return;
    }
  }
  hwt_frame_index = 0;
}

static void HWT101_ProcessByte(uint8_t data)
{
  uint8_t i;
  uint8_t checksum = 0;
  int16_t raw_value;

  if (hwt_frame_index == 0 && data != HWT101_FRAME_HEADER)
  {
    return;
  }

  hwt_frame[hwt_frame_index++] = data;
  if (hwt_frame_index < HWT101_FRAME_LENGTH)
  {
    return;
  }

  for (i = 0; i < HWT101_FRAME_LENGTH - 1; i++)
  {
    checksum = (uint8_t)(checksum + hwt_frame[i]);
  }
  if (checksum != hwt_frame[HWT101_FRAME_LENGTH - 1])
  {
    hwt_checksum_error_count++;
    HWT101_Resync();
    return;
  }

  hwt_frame_count++;
  if (hwt_frame[1] == HWT101_GYRO_FRAME_TYPE)
  {
    /* 0x52 Byte6/7：Z 轴角速度，低字节在前，量程 ±2000°/s。 */
    raw_value = (int16_t)((uint16_t)hwt_frame[6] | ((uint16_t)hwt_frame[7] << 8));
    imu_gyro_z = (float)raw_value / 32768.0f * 2000.0f;
    imu_gyro_last_tick = HAL_GetTick();
    hwt_gyro_count++;
  }
  else if (hwt_frame[1] == HWT101_ANGLE_FRAME_TYPE)
  {
    /* 只有校验正确的 0x53 帧才更新原始 yaw。 */
    raw_value = (int16_t)((uint16_t)hwt_frame[6] | ((uint16_t)hwt_frame[7] << 8));
    imu_yaw = (float)raw_value / 32768.0f * 180.0f;
    imu_yaw_valid = 1U;
    hwt_angle_count++;
  }
  hwt_frame_index = 0;
}

HAL_StatusTypeDef HWT101_Init(void)
{
  hwt_rx_count = 0;
  hwt_frame_count = 0;
  hwt_angle_count = 0;
  hwt_gyro_count = 0;
  hwt_last_rx_tick = 0;
  hwt_checksum_error_count = 0;
  hwt_rx_restart_error_count = 0;
  imu_yaw = 0;
  imu_gyro_z = 0;
  imu_gyro_last_tick = 0;
  imu_zero_yaw = 0;
  imu_yaw_valid = 0U;
  hwt_frame_index = 0;

  /* USART2 接收只在此处启动，main.c 不重复启动。 */
  return HAL_UART_Receive_IT(&huart2, &hwt_rx_byte, 1);
}

float IMU_GetYaw(void)
{
  if (imu_yaw_valid == 0U)
  {
    return 0.0f;
  }

  return HWT101_WrapAngle(-(imu_yaw - imu_zero_yaw));
}

void IMU_ZeroYaw(void)
{
  if (imu_yaw_valid != 0U)
  {
    imu_zero_yaw = imu_yaw;
  }
}

float IMU_GetGyroZ(void)
{
  return imu_gyro_z;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    Raspberry_RxCpltCallback();
    return;
  }

  if (huart->Instance != USART2)
  {
    return;
  }

  hwt_rx_count++;
  hwt_last_rx_tick = HAL_GetTick();
  HWT101_ProcessByte(hwt_rx_byte);

  /* 每个字节完成后立即挂起下一字节接收，回调内不延时。 */
  if (HAL_UART_Receive_IT(&huart2, &hwt_rx_byte, 1) != HAL_OK)
  {
    hwt_rx_restart_error_count++;
  }
}
