/**
  ******************************************************************************
  * @file    can.c
  * @brief   CAN1 外设配置
  ******************************************************************************
  * @note    保留 CubeMX 生成的 CAN 初始化参数；电机协议组帧位于 emm42.c。
  ******************************************************************************
  */

#include "can.h"
#include "emm42.h"

__IO CAN_t can = {0};
CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 14;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_4TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;

  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_CAN_MspInit(CAN_HandleTypeDef *canHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if (canHandle->Instance == CAN1)
  {
    __HAL_RCC_CAN1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef *canHandle)
{
  if (canHandle->Instance == CAN1)
  {
    __HAL_RCC_CAN1_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11 | GPIO_PIN_12);
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  }
}

void USER_CAN1_Filter_Init(void)
{
  CAN_FilterTypeDef filter_config;

  /* 32 位标识符屏蔽模式：只限定为扩展帧，不限定扩展 ID 数值。 */
  filter_config.FilterBank = 0;
  filter_config.FilterMode = CAN_FILTERMODE_IDMASK;
  filter_config.FilterScale = CAN_FILTERSCALE_32BIT;
  filter_config.FilterIdHigh = 0;
  filter_config.FilterIdLow = CAN_ID_EXT;
  filter_config.FilterMaskIdHigh = 0;
  filter_config.FilterMaskIdLow = CAN_ID_EXT;
  filter_config.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter_config.FilterActivation = ENABLE;
  filter_config.SlaveStartFilterBank = 0;

  while (HAL_CAN_ConfigFilter(&hcan1, &filter_config) != HAL_OK)
  {
  }
}

/*
 * CAN FIFO0 接收完成回调。
 * HAL 中断处理检测到 FIFO0 有消息后进入此函数；这里只处理 CAN1，
 * 读取一帧到现有用户缓存，再调用 X42S 解析函数更新电机反馈。
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  uint8_t i;

  if (hcan->Instance == CAN1)
  {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0,
                            (CAN_RxHeaderTypeDef *)&can.CAN_RxMsg,
                            (uint8_t *)can.rxData) == HAL_OK)
    {
      /* 清零未使用字节，防止上一帧残留数据影响协议解析。 */
      for (i = (uint8_t)can.CAN_RxMsg.DLC; i < 8; ++i)
        can.rxData[i] = 0;

      can.rxFrameFlag = true;
      EMM42_Process_CAN_Rx();
    }
  }
}
