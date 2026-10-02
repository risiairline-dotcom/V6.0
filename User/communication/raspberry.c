#include "raspberry.h"
#include "task_code.h"
#include "usart.h"

#include <string.h>

volatile char raspberry_rx_buffer[RASPBERRY_RX_BUFFER_SIZE];
volatile uint16_t raspberry_rx_length = 0U;
volatile uint32_t raspberry_rx_count = 0U;
volatile uint32_t usart3_rx_irq_count = 0U;
volatile uint32_t raspberry_rx_start_status = HAL_ERROR;
volatile uint32_t raspberry_rx_error_count = 0U;
volatile uint32_t raspberry_rx_drop_count = 0U;
volatile uint32_t raspberry_rx_overflow_count = 0U;
volatile uint8_t raspberry_rx_line_ready = 0U;
volatile uint32_t qr_valid_count = 0U;
volatile uint32_t qr_invalid_count = 0U;

static uint8_t raspberry_rx_byte;
static char raspberry_working_buffer[RASPBERRY_RX_BUFFER_SIZE];
static uint16_t raspberry_working_length;
static uint8_t raspberry_working_overflow;

void Raspberry_Init(void)
{
  raspberry_rx_length = 0U;
  raspberry_rx_count = 0U;
  usart3_rx_irq_count = 0U;
  raspberry_rx_error_count = 0U;
  raspberry_rx_drop_count = 0U;
  raspberry_rx_overflow_count = 0U;
  raspberry_rx_line_ready = 0U;
  raspberry_rx_buffer[0] = '\0';
  raspberry_working_length = 0U;
  raspberry_working_overflow = 0U;
  raspberry_working_buffer[0] = '\0';
  qr_valid_count = 0U;
  qr_invalid_count = 0U;

  raspberry_rx_start_status = HAL_UART_Receive_IT(&huart3, &raspberry_rx_byte, 1U);
  if (raspberry_rx_start_status != HAL_OK)
  {
    raspberry_rx_error_count++;
  }
}

void Raspberry_ProcessByte(uint8_t byte)
{
  uint16_t index;

  if (byte == '\r')
  {
    return;
  }

  if (byte == '\n')
  {
    if (raspberry_working_overflow == 0U)
    {
      if (raspberry_rx_line_ready == 0U)
      {
        for (index = 0U; index <= raspberry_working_length; index++)
        {
          raspberry_rx_buffer[index] = raspberry_working_buffer[index];
        }
        raspberry_rx_length = raspberry_working_length;
        raspberry_rx_line_ready = 1U;
      }
      else
      {
        raspberry_rx_drop_count++;
      }
    }
    raspberry_working_length = 0U;
    raspberry_working_overflow = 0U;
    raspberry_working_buffer[0] = '\0';
    return;
  }

  if (raspberry_working_overflow != 0U)
  {
    return;
  }

  if (raspberry_working_length < (RASPBERRY_RX_BUFFER_SIZE - 1U))
  {
    raspberry_working_buffer[raspberry_working_length] = (char)byte;
    raspberry_working_length++;
    raspberry_working_buffer[raspberry_working_length] = '\0';
  }
  else
  {
    raspberry_working_length = 0U;
    raspberry_working_buffer[0] = '\0';
    raspberry_working_overflow = 1U;
    raspberry_rx_overflow_count++;
    raspberry_rx_error_count++;
  }
}

void Raspberry_RxCpltCallback(void)
{
  usart3_rx_irq_count++;
  raspberry_rx_count++;
  Raspberry_ProcessByte(raspberry_rx_byte);

  if (HAL_UART_Receive_IT(&huart3, &raspberry_rx_byte, 1U) != HAL_OK)
  {
    raspberry_rx_error_count++;
  }
}

void Raspberry_Task(void)
{
  char line[RASPBERRY_RX_BUFFER_SIZE];
  uint16_t index;
  uint16_t length;

  if (raspberry_rx_line_ready == 0U)
  {
    return;
  }

  HAL_NVIC_DisableIRQ(USART3_IRQn);
  if (raspberry_rx_line_ready == 0U)
  {
    HAL_NVIC_EnableIRQ(USART3_IRQn);
    return;
  }
  length = raspberry_rx_length;
  for (index = 0U; index <= length; index++)
  {
    line[index] = raspberry_rx_buffer[index];
  }
  raspberry_rx_line_ready = 0U;
  HAL_NVIC_EnableIRQ(USART3_IRQn);

  if ((length == 18U) && (memcmp(line, "QR:", 3U) == 0) &&
      (TaskCode_Update(&line[3]) != 0U))
  {
    qr_valid_count++;
  }
  else
  {
    qr_invalid_count++;
  }
}
