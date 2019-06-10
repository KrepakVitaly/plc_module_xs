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

#ifdef STM32F405xx
  #define __bool_true_false_are_defined 1
  #include "stm32f4xx_hal.h"
#endif
#ifdef STM32F030x6
  #include "stm32f0xx_hal.h"
#endif


#ifdef __cplusplus
extern "C" {
#endif


#define PLC_UART_CYCLE_BUF_LEN 32
#define CIRC_BUFFER_OK 0
#define CIRC_BUFFER_ERR 1

typedef struct
{
  uint8_t data[PLC_UART_CYCLE_BUF_LEN];
  uint32_t max_size;
  uint32_t head;
  uint32_t tail;
  uint32_t size;
  
} CircularBuffer_Typedef;

typedef enum
{ 
  False = 0,  
  True = 1 
} Bool;

uint8_t CircularBuffer_Init(CircularBuffer_Typedef* buf);
uint8_t CircularBuffer_Clear(CircularBuffer_Typedef* buf);
Bool CircularBuffer_IsEmpty(CircularBuffer_Typedef* buf);
Bool CircularBuffer_IsFull(CircularBuffer_Typedef* buf);
uint8_t CircularBuffer_Put_OW(CircularBuffer_Typedef* buf, uint8_t val);
uint8_t CircularBuffer_Put(CircularBuffer_Typedef* buf, uint8_t val);
uint16_t CircularBuffer_GetLength(CircularBuffer_Typedef* buf);
uint8_t CircularBuffer_RemoveLastValue(CircularBuffer_Typedef* buf);
uint8_t CircularBuffer_RemoveLastNValues(CircularBuffer_Typedef* buf, 
                                         uint16_t num);
uint8_t CircularBuffer_GetLastValue(CircularBuffer_Typedef* buf, 
                                     uint8_t * result);
uint16_t CircularBuffer_GetLastNValues(CircularBuffer_Typedef* buf, 
                                       uint8_t * result, uint16_t num);
uint8_t CircularBuffer_RemoveFirstValue(CircularBuffer_Typedef* buf);
uint8_t CircularBuffer_RemoveFirstNValues(CircularBuffer_Typedef* buf, 
                                          uint16_t num);
uint8_t CircularBuffer_GetRandValue(CircularBuffer_Typedef* buf, uint16_t idx);

#ifdef __cplusplus
}
#endif

#endif // CIRCULAR_BUFFER_H_
