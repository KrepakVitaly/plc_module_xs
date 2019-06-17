/**
 * A simple header for reading the STM32 device UUID
 * Tested with STM32F4 and STM32F0 families
 * 
 * Version 1.0
 * Written by Uli Koehler
 * Published on http://techoverflow.net
 * Licensed under CC0 (public domain):
 * https://creativecommons.org/publicdomain/zero/1.0/
 */
#ifndef __UUID_H
#define __UUID_H
#include <stdint.h>
/**
 * The STM32 factory-programmed UUID memory.
 * Three values of 32 bits each starting at this address
 * Use like this: STM32_UUID[0], STM32_UUID[1], STM32_UUID[2]
 */
#define STM32_UUID ((uint32_t *)0x1FFFF7AC)


void foobar(); 

#endif //__UUID_H
