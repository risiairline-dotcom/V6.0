#ifndef RASPBERRY_H
#define RASPBERRY_H

#include <stdint.h>

#define RASPBERRY_RX_BUFFER_SIZE  128U

/* USART3 单字节中断接收观察量与完整行交接状态。 */
extern volatile char raspberry_rx_buffer[RASPBERRY_RX_BUFFER_SIZE];
extern volatile uint16_t raspberry_rx_length;
extern volatile uint32_t raspberry_rx_count;
extern volatile uint32_t usart3_rx_irq_count;
extern volatile uint32_t raspberry_rx_start_status;
extern volatile uint32_t raspberry_rx_error_count;
extern volatile uint32_t raspberry_rx_drop_count;
extern volatile uint32_t raspberry_rx_overflow_count;
extern volatile uint8_t raspberry_rx_line_ready;
extern volatile uint32_t qr_valid_count;
extern volatile uint32_t qr_invalid_count;

void Raspberry_Init(void);
void Raspberry_ProcessByte(uint8_t byte);
void Raspberry_RxCpltCallback(void);
void Raspberry_Task(void);

#endif /* RASPBERRY_H */
