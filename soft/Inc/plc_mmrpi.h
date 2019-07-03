/**
  ******************************************************************************
  * @file    
  * @author  Vitaliy KREPAK
  * @version V1.0.0
  * @date    
  * @brief   
  ******************************************************************************
  */


#ifndef PLC_MMRPI_H
#define PLC_MMRPI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f0xx_hal.h"

#define MY_ADDR_0 0x00
#define MY_ADDR_1 0x00
#define MY_ADDR_2 0x02


#define MY_ID 0x000001
#define MY_ID_0 (MY_ID & 0x0000FF)
#define MY_ID_1 ((MY_ID >> 8) & 0x0000FF)
#define MY_ID_2 ((MY_ID >> 16) & 0x0000FF)


#define CMD_SET_BRIGHTNESS 0x01


#define HEAD_BYTE_1 0x56
#define HEAD_BYTE_2 0x12
#define HEAD_BYTE_3 0x54

#define PACKET_SIZE 13

extern uint8_t plc_uart_buf;
extern uint8_t plc_uart_answer_ok[PACKET_SIZE];
extern uint8_t lamp_get_status[PACKET_SIZE];
extern uint32_t my_address;

#ifdef __cplusplus
}
#endif

#endif // PLC_MMRPI_H
