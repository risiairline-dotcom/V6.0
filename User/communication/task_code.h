#ifndef __TASK_CODE_H__
#define __TASK_CODE_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  uint8_t batch1_color[3];
  uint8_t batch1_position[3];
  uint8_t batch2_color[3];
  uint8_t batch2_process_position[3];
} TaskCodeData_t;

/* 仅在完整任务码校验并解析成功后为 1。 */
extern volatile uint8_t qr_task_ready;

void TaskCode_Init(void);
void TaskCode_ClearReady(void);
uint8_t TaskCode_IsReady(void);
uint8_t TaskCode_Update(const char *code);
const char *TaskCode_Get(void);
const TaskCodeData_t *TaskCode_GetData(void);

#ifdef __cplusplus
}
#endif

#endif /* __TASK_CODE_H__ */
