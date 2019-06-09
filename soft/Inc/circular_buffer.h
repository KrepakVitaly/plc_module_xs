/**
  ******************************************************************************
  * @file    
  * @author  Vitaliy KREPAK
  * @version V1.0.0
  * @date    
  * @brief   
  ******************************************************************************
  */
  
#ifndef CIRCULAR_BUFFER_H_
#define CIRCULAR_BUFFER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f0xx_hal.h"

typedef enum { false, true } bool;

#define PLC_UART_CYCLE_BUF_LEN 1024
#define CIRC_BUFFER_OK 0
#define CIRC_BUFFER_ERR 1

extern uint8_t plc_uart_cycle_buf[PLC_UART_CYCLE_BUF_LEN];
extern uint32_t plc_circular_buf_data_size;
extern uint32_t plc_circular_buf_start;
extern uint32_t plc_circular_buf_end;

typedef struct
{
  uint8_t data[PLC_UART_CYCLE_BUF_LEN];
  uint32_t max_size;
  uint32_t head;
  uint32_t tail;
  uint32_t size;
  
} CircularBuffer_Typedef;


uint8_t CircularBuffer_Init(CircularBuffer_Typedef* buf);
uint8_t CircularBuffer_Clear(CircularBuffer_Typedef* buf);
bool CircularBuffer_IsEmpty(CircularBuffer_Typedef* buf);
bool CircularBuffer_IsFull(CircularBuffer_Typedef* buf);
uint8_t CircularBuffer_Put_OW(CircularBuffer_Typedef* buf, uint8_t val);
uint8_t CircularBuffer_Put(CircularBuffer_Typedef* buf, uint8_t val);
uint16_t CircularBuffer_GetLength(CircularBuffer_Typedef* buf);
uint8_t CircularBuffer_RemoveLastValue(CircularBuffer_Typedef* buf);
uint8_t CircularBuffer_RemoveLastNValues(CircularBuffer_Typedef* buf, 
                                         uint16_t num);
uint8_t CircularBuffer_GetLastValue(CircularBuffer_Typedef* buf, 
                                     uint8_t * result);
uint8_t CircularBuffer_RemoveFirstValue(CircularBuffer_Typedef* buf);
uint8_t CircularBuffer_RemoveFirstNValues(CircularBuffer_Typedef* buf, 
                                          uint16_t num);


#ifdef __cplusplus
}
#endif

#endif // CIRCULAR_BUFFER_H_
