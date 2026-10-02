/**
  ******************************************************************************
  * @file    can.h
  * @brief   CAN1 外设配置及接收缓存声明
  ******************************************************************************
  */

#ifndef __can_H
#define __can_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>

/* CAN 接收缓存，供中断接收函数使用。 */
typedef struct
{
  __IO CAN_RxHeaderTypeDef CAN_RxMsg;
  __IO uint8_t rxData[32];
  __IO bool rxFrameFlag;
} CAN_t;

extern CAN_HandleTypeDef hcan1;
extern __IO CAN_t can;

void MX_CAN1_Init(void);

/* 配置为接收所有扩展帧，X42S 返回帧使用扩展帧 ID。 */
void USER_CAN1_Filter_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __can_H */
