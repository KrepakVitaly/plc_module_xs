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
