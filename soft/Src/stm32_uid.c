#include "stm32_uid.h"

void foobar() {
    uint32_t idPart1 = STM32_UUID[0];
    uint32_t idPart2 = STM32_UUID[1];
    uint32_t idPart3 = STM32_UUID[2];
    //do something with the overall 96 bits
}
