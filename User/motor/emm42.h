/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : emm42.h
  * @brief          : 张大头 X42S_V1.0（Emm 固件）CAN 协议接口
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __EMM42_H
#define __EMM42_H

#include "can.h"

#ifdef __cplusplus
extern "C" {
#endif

/* X42S CAN 协议中的方向定义：00=CW，01=CCW。 */
#define EMM42_DIR_CW             0x00
#define EMM42_DIR_CCW            0x01

/* 同步标志：0=立即执行，1=缓存命令等待同步。 */
#define EMM42_SYNC_NOW           0x00
#define EMM42_SYNC_BUFFER        0x01

/* Emm 位置模式运动方式。 */
#define EMM42_POS_RELATIVE       0x00
#define EMM42_POS_ABSOLUTE       0x01

/* 多机同步触发使用广播地址 0。 */
#define EMM42_BROADCAST_ADDR     0x00

/* [DEBUG] 位置协议快照：保存传入参数及实际拆出的两帧 CAN 数据。 */
typedef struct
{
  uint8_t id;
  uint8_t dir;
  uint16_t speed_rpm;
  uint8_t acc;
  uint32_t pulse;
  uint8_t motion_mode;
  uint8_t sync;
  uint8_t packet0[8];
  uint8_t packet1[5];
} EMM42PositionDebug_t;

extern volatile EMM42PositionDebug_t emm42_position_debug;

/* [DEBUG] 最近一次停止命令的地址和4字节CAN数据，供Keil Watch观察。 */
extern volatile uint8_t emm42_debug_stop_id;
extern volatile uint8_t emm42_debug_stop_data[4];

/* 电机使能控制：state=true 锁轴，state=false 关闭使能。 */
void EMM42_Enable(uint8_t addr, bool state, bool sync_flag);

/* Emm 固件速度模式：速度单位 RPM，加速度为 0~255 档。 */
void EMM42_VelocityControl(uint8_t addr, uint8_t dir, uint16_t speed_rpm, uint8_t acc, bool sync_flag);

/* 位置模式：pulse 为脉冲数，默认 16 细分时 3200 脉冲约为一圈。 */
void EMM42_PositionControl(uint8_t addr, uint8_t dir, uint16_t speed_rpm, uint8_t acc, uint32_t pulse, uint8_t motion_mode, bool sync_flag);

/* 面向应用的停止函数。 */
void EMM42_Stop(uint8_t addr);

/* 多机同步执行：0=立即触发，1=保持已缓存命令。 */
void EMM42_Multi_Motor_Cmd(uint8_t sync_flag);

/*
 * 读取电机反馈。
 * 读取函数会发送查询命令，并返回最近一次接收并解析的数据。
 * 由于 CAN 返回是异步的，第一次调用时可能仍返回上一次缓存值。
 */
uint8_t EMM42_Read_Status(uint8_t addr);
int32_t EMM42_Read_Position(uint8_t addr);
int16_t EMM42_Read_Speed(uint8_t addr);

/* 请求位置并等待该地址收到一帧新反馈，超时返回 false。 */
bool EMM42_Read_Position_Confirmed(uint8_t addr, int32_t *position, uint32_t timeout_ms);

/* CAN 接收中断收到一帧数据后调用，负责校验并更新反馈缓存。 */
void EMM42_Process_CAN_Rx(void);

#ifdef __cplusplus
}
#endif

#endif /* __EMM42_H */
