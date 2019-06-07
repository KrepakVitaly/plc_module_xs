/**
  ******************************************************************************
  * @file    
  * @author  Vitaliy KREPAK
  * @version V1.0.0
  * @date    
  * @brief   
  ******************************************************************************
  */
  
#include "circular_buffer.h"
  
uint8_t plc_uart_cycle_buf[PLC_UART_CYCLE_BUF_LEN] = {0};
uint32_t plc_circular_buf_data_size;
uint32_t plc_circular_buf_start;
uint32_t plc_circular_buf_end;

typedef struct
{
  uint8_t data[PLC_UART_CYCLE_BUF_LEN];
  uint32_t max_size;
  uint32_t head;
  uint32_t tail;
  uint32_t size;
  bool full;
  
} CircularBuffer_Typedef;


/// Pass in a storage buffer and size 
/// Returns a circular buffer handle
uint8_t init(CircularBuffer_Typedef* buf, uint16_t size)
{
  
}

/// Free a circular buffer, head == tail
uint8_t clear(CircularBuffer_Typedef* buf)
{
  
}

/// Put version 1 continues to add data if the buffer is full
/// Old data is overwritten
uint8_t put(CircularBuffer_Typedef* buf, uint8_t val)
{
  
}

/// Put Version 2 rejects new data if the buffer is full
/// Returns 0 on success, -1 if buffer is full
uint8_t put2(CircularBuffer_Typedef* buf, uint8_t val)
{
  
}


uint8_t remove_last_value(CircularBuffer_Typedef* buf, uint8_t val)
{
  
}

uint16_t get_len(CircularBuffer_Typedef* buf)
{
}

uint8_t remove_last_n_values(CircularBuffer_Typedef* buf, uint8_t val, uint16_t num)
{
  
}

uint8_t append_n_values(CircularBuffer_Typedef* buf, uint8_t val, uint16_t num)
{
  
}

uint16_t get_last_n_values(CircularBuffer_Typedef* buf, uint16_t num)
{
}

uint8_t is_empty(CircularBuffer_Typedef* buf)
{
}

uint8_t is_full(CircularBuffer_Typedef* buf)
{
}
