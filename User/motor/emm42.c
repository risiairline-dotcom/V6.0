/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : emm42.c
  * @brief          : 张大头 X42S_V1.0（Emm 固件）CAN 协议实现
  ******************************************************************************
  * @note
  *   本文件严格按张大头 X42S CAN 通讯格式组帧：
  *   1. 使用扩展帧；帧 ID=(电机地址<<8)|分包序号。
  *   2. 每个 CAN 数据包的第一个字节都是功能码。
  *   3. 默认校验码固定为 0x6B，不改用其他校验算法。
  ******************************************************************************
  */
/* USER CODE END Header */

#include "emm42.h"
#include <stddef.h>

/* 张大头协议默认校验字节。 */
#define EMM42_CHECK_BYTE        0x6B

/* 张大头官方读取命令码。 */
#define EMM42_READ_SPEED_CMD    0x35
#define EMM42_READ_POSITION_CMD 0x36
#define EMM42_READ_STATUS_CMD   0x3A

/* 电机地址为 8 位，按地址保存最近一次反馈值。 */
typedef struct
{
  volatile int32_t position;
  volatile int16_t speed;
  volatile uint8_t status;
  volatile bool position_valid;
  volatile uint32_t position_update_count;
} EMM42_Feedback_t;

static volatile EMM42_Feedback_t emm42_feedback[256] = {0};
/* [DEBUG] 以下变量只保存协议和停止命令快照，不参与控制判断。 */
volatile EMM42PositionDebug_t emm42_position_debug = {0};
volatile uint8_t emm42_debug_stop_id = 0;
volatile uint8_t emm42_debug_stop_data[4] = {0};

/*
 * 发送一条张大头命令。
 * command 的格式为：功能码 + 命令数据 + 0x6B，地址单独由参数传入。
 * 当命令超过单帧 8 字节时，按官方例程每包重复发送功能码，
 * 并用 (addr<<8)|packet 作为扩展帧 ID。
 */
static void EMM42_SendCommand(uint8_t addr, const uint8_t *command, uint8_t command_len)
{
  CAN_TxHeaderTypeDef tx_header = {0};
  uint8_t tx_data[8] = {0};
  uint32_t tx_mailbox = 0;
  uint32_t tx_ok_mask;
  uint32_t wait_start;
  uint8_t data_index = 1;
  uint8_t packet = 0;
  uint8_t remaining;
  uint8_t copy_len;
  uint8_t retry;
  uint8_t i;
  HAL_StatusTypeDef send_status;

  /* 至少应包含功能码和校验字节。 */
  if ((command == NULL) || (command_len < 2))
  {
    return;
  }

  /* 功能码之后的内容包含命令数据和最后的 0x6B。 */
  remaining = (uint8_t)(command_len - 1);

  tx_header.StdId = 0x00;
  tx_header.IDE = CAN_ID_EXT;
  tx_header.RTR = CAN_RTR_DATA;

  while (remaining > 0)
  {
    /* 张大头 CAN 协议要求每一个分包都带功能码。 */
    tx_data[0] = command[0];
    copy_len = (remaining > 7) ? 7 : remaining;

    for (i = 0; i < copy_len; ++i)
    {
      tx_data[i + 1] = command[data_index + i];
    }

    tx_header.ExtId = ((uint32_t)addr << 8) | packet;
    tx_header.DLC = (uint32_t)(copy_len + 1);

    send_status = HAL_ERROR;
    for (retry = 0; retry < 3; ++retry)
    {
      /* 连续发送前先确认至少有一个空闲邮箱，避免覆盖或遗漏待发报文。 */
      wait_start = HAL_GetTick();
      while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0)
      {
        if ((HAL_GetTick() - wait_start) >= 20)
        {
          break;
        }
      }

      if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0)
      {
        continue;
      }

      send_status = HAL_CAN_AddTxMessage(&hcan1,&tx_header,tx_data,&tx_mailbox);
      if (send_status != HAL_OK)
      {
        continue;
      }

      /* 等待本邮箱发送请求结束，确保当前分包完成后再发送下一分包。 */
      wait_start = HAL_GetTick();
      while (HAL_CAN_IsTxMessagePending(&hcan1,tx_mailbox) != 0)
      {
        if ((HAL_GetTick() - wait_start) >= 20)
        {
          HAL_CAN_AbortTxRequest(&hcan1,tx_mailbox);
          send_status = HAL_TIMEOUT;
          break;
        }
      }

      if (send_status != HAL_OK)
      {
        continue;
      }

      if (tx_mailbox == CAN_TX_MAILBOX0)
      {
        tx_ok_mask = CAN_TSR_TXOK0;
      }
      else if (tx_mailbox == CAN_TX_MAILBOX1)
      {
        tx_ok_mask = CAN_TSR_TXOK1;
      }
      else
      {
        tx_ok_mask = CAN_TSR_TXOK2;
      }

      /* TXOK 置位才表示报文成功发到总线；失败时重新提交当前分包。 */
      if ((hcan1.Instance->TSR & tx_ok_mask) != 0)
      {
        break;
      }
      send_status = HAL_ERROR;
    }

    /* 当前分包重试后仍失败，停止本条命令，避免后续分包形成残缺协议帧。 */
    if (send_status != HAL_OK)
    {
      return;
    }

    data_index = (uint8_t)(data_index + copy_len);
    remaining = (uint8_t)(remaining - copy_len);
    ++packet;
  }
}

/*
 * 使能命令：Addr F3 AB 00/01 00/01 6B
 * state：0=不使能（松轴），1=使能（锁轴）。
 */
void EMM42_Enable(uint8_t addr, bool state, bool sync_flag)
{
  const uint8_t command[] =
  {
    0xF3, 0xAB, (uint8_t)state, (uint8_t)sync_flag, EMM42_CHECK_BYTE
  };

  EMM42_SendCommand(addr,command,(uint8_t)sizeof(command));
}

/*
 * Emm 速度模式命令：Addr F6 方向 速度_H 速度_L 加速度 同步标志 6B
 * 方向：00=CW，01=CCW；速度范围按手册为 0~3000 RPM。
 */
void EMM42_VelocityControl(uint8_t addr, uint8_t dir, uint16_t speed_rpm, uint8_t acc, bool sync_flag)
{
  const uint8_t command[] =
  {
    0xF6,
    dir,
    (uint8_t)(speed_rpm >> 8),
    (uint8_t)(speed_rpm >> 0),
    acc,
    (uint8_t)sync_flag,
    EMM42_CHECK_BYTE
  };

  EMM42_SendCommand(addr,command,(uint8_t)sizeof(command));
}

/*
 * Emm 位置模式命令：
 * Addr FD 方向 速度_H 速度_L 加速度 脉冲数(4 字节) 运动模式 同步标志 6B
 */
void EMM42_PositionControl(uint8_t addr, uint8_t dir, uint16_t speed_rpm, uint8_t acc, uint32_t pulse, uint8_t motion_mode, bool sync_flag)
{
  const uint8_t command[] =
  {
    0xFD,
    dir,
    (uint8_t)(speed_rpm >> 8),
    (uint8_t)(speed_rpm >> 0),
    acc,
    (uint8_t)(pulse >> 24),
    (uint8_t)(pulse >> 16),
    (uint8_t)(pulse >> 8),
    (uint8_t)(pulse >> 0),
    motion_mode,
    (uint8_t)sync_flag,
    EMM42_CHECK_BYTE
  };

  /*
   * X42S 经典 CAN 分包结果可在 Watch 中查看：
   * packet0 = FD DIR SPEED_H SPEED_L ACC PULSE_3 PULSE_2 PULSE_1
   * packet1 = FD PULSE_0 MODE SYNC 6B；对应扩展帧 ID 分别为 addr<<8 和 (addr<<8)|1。
   */
  emm42_position_debug.id = addr;
  emm42_position_debug.dir = dir;
  emm42_position_debug.speed_rpm = speed_rpm;
  emm42_position_debug.acc = acc;
  emm42_position_debug.pulse = pulse;
  emm42_position_debug.motion_mode = motion_mode;
  emm42_position_debug.sync = (uint8_t)sync_flag;
  emm42_position_debug.packet0[0] = command[0];
  emm42_position_debug.packet0[1] = command[1];
  emm42_position_debug.packet0[2] = command[2];
  emm42_position_debug.packet0[3] = command[3];
  emm42_position_debug.packet0[4] = command[4];
  emm42_position_debug.packet0[5] = command[5];
  emm42_position_debug.packet0[6] = command[6];
  emm42_position_debug.packet0[7] = command[7];
  emm42_position_debug.packet1[0] = command[0];
  emm42_position_debug.packet1[1] = command[8];
  emm42_position_debug.packet1[2] = command[9];
  emm42_position_debug.packet1[3] = command[10];
  emm42_position_debug.packet1[4] = command[11];

  EMM42_SendCommand(addr,command,(uint8_t)sizeof(command));
}

/* 立即停止命令：Addr FE 98 00 6B。 */
void EMM42_Stop(uint8_t addr)
{
  const uint8_t command[] =
  {
    0xFE, 0x98, EMM42_SYNC_NOW, EMM42_CHECK_BYTE
  };

  /* 保存最近一次停止命令，确认地址4收到的帧内容是否正确。 */
  emm42_debug_stop_id = addr;
  emm42_debug_stop_data[0] = command[0];
  emm42_debug_stop_data[1] = command[1];
  emm42_debug_stop_data[2] = command[2];
  emm42_debug_stop_data[3] = command[3];

  EMM42_SendCommand(addr,command,(uint8_t)sizeof(command));
}

/*
 * 多机同步执行命令：广播发送 00 FF 66 6B。
 * 速度或位置命令的 sync=1 时先缓存，传入 0 后立即触发同步执行；
 * 传入 1 时保持缓存状态，不发送触发命令。
 */
void EMM42_Multi_Motor_Cmd(uint8_t sync_flag)
{
  const uint8_t command[] = {0xFF,0x66,EMM42_CHECK_BYTE};

  if (sync_flag == EMM42_SYNC_NOW)
  {
    EMM42_SendCommand(EMM42_BROADCAST_ADDR,command,(uint8_t)sizeof(command));
  }
}

/*
 * 读取电机状态标志。
 * 发送格式：Addr 3A 6B。
 * 返回值为最近一次响应中的状态字节。
 */
uint8_t EMM42_Read_Status(uint8_t addr)
{
  const uint8_t command[] = {EMM42_READ_STATUS_CMD,EMM42_CHECK_BYTE};

  EMM42_SendCommand(addr,command,(uint8_t)sizeof(command));
  return emm42_feedback[addr].status;
}

/*
 * 读取电机实时位置。
 * 发送格式：Addr 36 6B。
 * 返回格式中的符号字节为 0 正、1 负，位置数值为 4 字节大端无符号幅值。
 */
int32_t EMM42_Read_Position(uint8_t addr)
{
  const uint8_t command[] = {EMM42_READ_POSITION_CMD,EMM42_CHECK_BYTE};

  EMM42_SendCommand(addr,command,(uint8_t)sizeof(command));
  return emm42_feedback[addr].position;
}

/*
 * 记录查询前的更新计数，只有接收中断解析到该电机的新位置反馈后才返回。
 * 该接口用于复位后的基准同步和相对动作起点确认，不用于高频到位轮询。
 */
bool EMM42_Read_Position_Confirmed(uint8_t addr, int32_t *position, uint32_t timeout_ms)
{
  uint32_t update_count;
  uint32_t wait_start;

  if ((position == NULL) || (timeout_ms == 0))
  {
    return false;
  }

  update_count = emm42_feedback[addr].position_update_count;
  wait_start = HAL_GetTick();
  EMM42_Read_Position(addr);

  while (emm42_feedback[addr].position_update_count == update_count)
  {
    if ((HAL_GetTick() - wait_start) >= timeout_ms)
    {
      return false;
    }
  }

  if (!emm42_feedback[addr].position_valid)
  {
    return false;
  }

  *position = emm42_feedback[addr].position;
  return true;
}

/*
 * 读取电机实时速度。
 * 发送格式：Addr 35 6B。
 * 返回格式中的符号字节为 0 正、1 负，速度数值为 2 字节大端无符号幅值。
 */
int16_t EMM42_Read_Speed(uint8_t addr)
{
  const uint8_t command[] = {EMM42_READ_SPEED_CMD,EMM42_CHECK_BYTE};

  EMM42_SendCommand(addr,command,(uint8_t)sizeof(command));
  return emm42_feedback[addr].speed;
}

/*
 * 解析 CAN 接收缓存中的一帧反馈数据。
 *
 * CAN 扩展帧 ID 的高位部分为电机地址，低 8 位为分包序号；
 * 读取反馈均为单帧响应，因此这里只处理分包序号 0。
 * CAN 数据区不再包含地址，格式为：功能码 + 数据 + 0x6B。
 */
void EMM42_Process_CAN_Rx(void)
{
  uint8_t addr;
  uint8_t function;
  uint8_t data_len;
  uint16_t speed_value;
  uint32_t position_value;
  int32_t signed_position;

  if (!can.rxFrameFlag)
  {
    return;
  }

  data_len = (uint8_t)can.CAN_RxMsg.DLC;
  can.rxFrameFlag = false;

  /* 只解析 X42S 使用的扩展数据帧，并过滤非第 0 个分包。 */
  if ((can.CAN_RxMsg.IDE != CAN_ID_EXT) ||
      ((can.CAN_RxMsg.ExtId & 0xFF) != 0) ||
      (data_len == 0) ||
      (data_len > 8))
  {
    return;
  }

  /* 官方协议最后一个字节固定为 0x6B。 */
  if (can.rxData[data_len - 1] != EMM42_CHECK_BYTE)
  {
    return;
  }

  addr = (uint8_t)(can.CAN_RxMsg.ExtId >> 8);
  function = can.rxData[0];

  switch (function)
  {
    case EMM42_READ_STATUS_CMD:
      /* 响应格式：Addr 3A 状态字节 6B。 */
      if (data_len >= 3)
      {
        emm42_feedback[addr].status = can.rxData[1];
      }
      break;

    case EMM42_READ_SPEED_CMD:
      /* 响应格式：Addr 35 符号 速度_H 速度_L 6B。 */
      if (data_len >= 5)
      {
        speed_value = (uint16_t)(((uint16_t)can.rxData[2] << 8) | can.rxData[3]);
        emm42_feedback[addr].speed = (can.rxData[1] == 0) ? (int16_t)speed_value : -(int16_t)speed_value;
      }
      break;

    case EMM42_READ_POSITION_CMD:
      /* 响应格式：Addr 36 符号 位置_H 位置_MH 位置_ML 位置_L 6B。 */
      if (data_len >= 7)
      {
        position_value = ((uint32_t)can.rxData[2] << 24) |
                         ((uint32_t)can.rxData[3] << 16) |
                         ((uint32_t)can.rxData[4] << 8) |
                         can.rxData[5];
        signed_position = (int32_t)position_value;
        emm42_feedback[addr].position = (can.rxData[1] == 0) ? signed_position : -signed_position;
        /* 位置写入完成后再置有效并递增计数，供同步读取判断是否收到新反馈。 */
        emm42_feedback[addr].position_valid = true;
        emm42_feedback[addr].position_update_count++;
      }
      break;

    default:
      /* 其他返回帧不是本次反馈接口需要的数据。 */
      break;
  }
}
