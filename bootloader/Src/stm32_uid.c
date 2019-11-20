#include "stm32_uid.h"

STM32_UUID Signature;

void Init_UUID(void)
{
  Signature.idPart1 = STM32_UUID_ADDR[0];
  Signature.idPart2 = STM32_UUID_ADDR[1];
  Signature.idPart3 = STM32_UUID_ADDR[2];
  
  Signature.x = (Signature.idPart1 >> 16) & 0xFFFF;  // x-coordinate
  Signature.y = Signature.idPart1 & 0xFFFF;          // y-coordinate
  Signature.Wafer = Signature.idPart2 >> 0;          // Wafer number
  
  Signature.Lot[0] = (Signature.idPart2 >> 8 ) & 0xFF;  // Lot number
  Signature.Lot[1] = (Signature.idPart2 >> 16) & 0xFF;  // Lot number
  Signature.Lot[2] = (Signature.idPart2 >> 24) & 0xFF;  // Lot number
  Signature.Lot[3] = (Signature.idPart3 >> 0 ) & 0xFF;  // Lot number
  Signature.Lot[4] = (Signature.idPart3 >> 8 ) & 0xFF;  // Lot number
  Signature.Lot[5] = (Signature.idPart3 >> 16) & 0xFF;  // Lot number
  Signature.Lot[6] = (Signature.idPart3 >> 24) & 0xFF;  // Lot number
  
  Signature.flash_size = ((FLASH_SIZE_ADDR[0] >> 8) & 0xFF) + ((FLASH_SIZE_ADDR[0] << 8) & 0xFF00); //size in kbytes
}
