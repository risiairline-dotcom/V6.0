#ifndef __TJC_H__
#define __TJC_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* [DEBUG/DORMANT] 阶段1未初始化USART6，发送计数仅为预留观察量。 */
extern volatile uint32_t tjc_send_count;

void TJC_SendStr(const char *str);
void TJC_SendByte(uint8_t byte);
void TJC_Init(void);
void TJC_ShowTaskCode(const char *task_code);

#ifdef __cplusplus
}
#endif

#endif /* __TJC_H__ */
