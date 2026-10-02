#include "task_code.h"
#include "tjc.h"

#include <string.h>

static char task_code[16];
static TaskCodeData_t task_code_data;
volatile uint8_t qr_task_ready = 0U;

void TaskCode_Init(void)
{
  task_code[0] = '\0';
  memset(&task_code_data, 0, sizeof(task_code_data));
  qr_task_ready = 0U;
}

void TaskCode_ClearReady(void)
{
  qr_task_ready = 0U;
}

uint8_t TaskCode_IsReady(void)
{
  return qr_task_ready;
}

uint8_t TaskCode_Update(const char *code)
{
  uint8_t index;

  if ((code == NULL) || (strlen(code) != 15))
  {
    return 0;
  }

  for (index = 0; index < 15; index++)
  {
    if ((index == 3) || (index == 7) || (index == 11))
    {
      if (code[index] != '+')
      {
        return 0;
      }
    }
    else if ((code[index] < '0') || (code[index] > '9'))
    {
      return 0;
    }

    if (((index < 3U) || ((index >= 8U) && (index <= 10U))) &&
        ((code[index] < '1') || (code[index] > '6')))
    {
      return 0;
    }
    if ((((index >= 4U) && (index <= 6U)) || (index >= 12U)) &&
        ((code[index] < '1') || (code[index] > '3')))
    {
      return 0;
    }
  }

  memcpy(task_code, code, sizeof(task_code));
  for (index = 0; index < 3U; index++)
  {
    task_code_data.batch1_color[index] = (uint8_t)(code[index] - '0');
    task_code_data.batch1_position[index] = (uint8_t)(code[index + 4U] - '0');
    task_code_data.batch2_color[index] = (uint8_t)(code[index + 8U] - '0');
    task_code_data.batch2_process_position[index] = (uint8_t)(code[index + 12U] - '0');
  }
  qr_task_ready = 1U;
  TJC_ShowTaskCode(task_code);

  return 1;
}

const char *TaskCode_Get(void)
{
  return task_code;
}

const TaskCodeData_t *TaskCode_GetData(void)
{
  return &task_code_data;
}
