#include "plc_mmrpi.h"

uint8_t plc_uart_buf = 0;


//                                SIZE| HEAD OF PACKET |             ADDRESS             | STAT | LEVEL |   CRC32 (not used)  |
//                                  0     1     2    3      4            5           6      7     8     9     10   11    12
uint8_t plc_uart_answer_ok[13] = {0x0c, 0x62, 0xAA, 0x77, MY_ADDR_0, MY_ADDR_1, MY_ADDR_2, 0x00, 0x0a, 0x0c, 0x26, 0x68, 0x68};

//                                      SIZE| HEAD OF PACKET |     ADDRESS    |  CMD  | DATA |   CRC32 (not used)  |
//                                       0     1     2    3      4    5     6      7     8     9     10   11    12
uint8_t lamp_get_status[PACKET_SIZE] = {0x0c ,0x56 ,0x12 ,0x54 ,0x00 ,0x00 ,0x01 ,0x02 ,0x00 ,0xec ,0xf3 ,0x81 ,0x8b};

uint32_t my_address = (MY_ADDR_0 << 16) + (MY_ADDR_1 << 8) + MY_ADDR_2;
