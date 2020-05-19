#include "dali_interface_lib.h"
#include "main.h"

uint8_t dali_cntr;
uint32_t dali_cmd = 0x01FE01; //0000 0001 - start, 0 000 000 0 0001 0001
uint32_t dali_cmd_sh = 0x01FE01; //0000 0001 - start, 1 111 111 0 0000 000


uint8_t step_DALI_set_brightness(uint8_t * dali_cntr)
{
//    if (*dali_cntr >= 34)
//    {
//      *dali_cntr = 0;
//      //HAL_GPIO_WritePin(DALI_TX_GPIO_Port, DALI_TX_Pin, GPIO_PIN_RESET);
//      return 0;
//    }
//    
//    if (*dali_cntr % 2 == 0)
//    {
//      if( (((dali_cmd) >> (17 - (*dali_cntr)/2 - 1)) & 0x01) == 1 )
//        HAL_GPIO_WritePin(DALI_TX_GPIO_Port, DALI_TX_Pin, GPIO_PIN_SET);
//      else
//        HAL_GPIO_WritePin(DALI_TX_GPIO_Port, DALI_TX_Pin, GPIO_PIN_RESET);
//    }
//    else
//    {
//      HAL_GPIO_TogglePin(DALI_TX_GPIO_Port, DALI_TX_Pin);
//    }
//    
//    (*dali_cntr)++;
//    return 1;
}
