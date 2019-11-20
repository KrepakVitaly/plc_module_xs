/**
 * A simple header for reading the STM32 device UUID
 * Tested with STM32F4 and STM32F0 families
 * 
 * Version 1.0
 * Written by Uli Koehler
 * Published on http://techoverflow.net
 * Licensed under CC0 (public domain):
 * https://creativecommons.org/publicdomain/zero/1.0/
 * http://blog.gorski.pm/stm32-unique-id
 * https://stackoverflow.com/questions/29323699/how-to-find-the-device-id-of-a-stm32f103ret
 * https://gist.github.com/ElectronicaXAB3
 */
#ifndef __UUID_H
#define __UUID_H
#include <stdint.h>

/**
 * The STM32 factory-programmed UUID memory.
 * Three values of 32 bits each starting at this address
 * Use like this: STM32_UUID[0], STM32_UUID[1], STM32_UUID[2]
 */
 
#ifdef STM32F031x6
#define STM32_UUID_ADDR ((uint32_t *)0x1FFFF7AC)
#define FLASH_SIZE_ADDR  ((uint32_t *)0x1FFFF7CC)
#endif

typedef struct {
    uint16_t x;  // x-coordinate
    uint16_t y;  // y-coordinate
    uint8_t Wafer;  // Wafer number
    char Lot[7];  // Lot number
    
    uint32_t idPart1;
    uint32_t idPart2 ;
    uint32_t idPart3;
  
    uint16_t flash_size;
} STM32_UUID;

extern STM32_UUID Signature;

void Init_UUID(); 

#endif //__UUID_H
