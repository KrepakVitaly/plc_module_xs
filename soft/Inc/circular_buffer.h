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


#define PLC_UART_CYCLE_BUF_LEN 1024

extern uint8_t plc_uart_cycle_buf[PLC_UART_CYCLE_BUF_LEN];
extern uint32_t plc_circular_buf_data_size;
extern uint32_t plc_circular_buf_start;
extern uint32_t plc_circular_buf_end;



#ifdef __cplusplus
}
#endif

#endif // CIRCULAR_BUFFER_H_
