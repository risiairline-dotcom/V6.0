#include "tjc.h"
#include "usart.h"

#include <stdio.h>
#include <string.h>

volatile uint32_t tjc_send_count = 0;

static void TJC_SendCommand(const char *cmd)
{
  if (cmd == NULL)
  {
    return;
  }

  TJC_SendStr(cmd);
  TJC_SendByte(0xFF);
  TJC_SendByte(0xFF);
  TJC_SendByte(0xFF);
}

void TJC_SendStr(const char *str)
{
  if (str == NULL)
  {
    return;
  }

  HAL_UART_Transmit(&huart6, (uint8_t *)str, (uint16_t)strlen(str), HAL_MAX_DELAY);
  tjc_send_count++;
}

void TJC_SendByte(uint8_t byte)
{
  HAL_UART_Transmit(&huart6, &byte, 1, HAL_MAX_DELAY);
}

void TJC_Init(void)
{
  /* 等待陶晶驰屏幕完成上电启动。 */
  HAL_Delay(1000);

  /* 清理屏幕上电期间可能收到的异常串口数据。 */
  TJC_SendByte(0x00);
  TJC_SendByte(0xFF);
  TJC_SendByte(0xFF);
  TJC_SendByte(0xFF);

  HAL_Delay(100);

  /* 切换到任务码显示页面。 */
  TJC_SendCommand("page page0");

  HAL_Delay(100);
}

void TJC_ShowTaskCode(const char *task_code)
{
  char command[32];
  int length;

  if (task_code == NULL)
  {
    return;
  }

  length = snprintf(command, sizeof(command), "t0.txt=\"%s\"", task_code);
  if ((length < 0) || ((size_t)length >= sizeof(command)))
  {
    return;
  }

  TJC_SendCommand(command);
}
