/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "crc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32_uid.h"
#include "plc_greenlight_protocol.h"
#include "stm32f0xx_hal_plc_uart.h"
#include "circular_buffer.h"
#include "plc_mmrpi.h"
#include "dali_interface_lib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t status_g = 0;
uint8_t brightness_g = 0;
uint8_t volt_g = 0;
uint8_t amps_g = 0;
uint8_t temp_g = 0;

CircularBuffer_Typedef kq130_buf;
uint8_t packet_analyze_buf[PACKET_SIZE];
uint8_t new_byte_received = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
 
/**Force VectorTable to specific memory position defined in linker*/
volatile uint32_t VectorTable[48] __attribute__((section(".RAMVectorTable")));

void remapMemToSRAM( void )
{
    uint32_t vecIndex = 0;
    __disable_irq();
 
    for(vecIndex = 0; vecIndex < 48; vecIndex++){
        VectorTable[vecIndex] = *(volatile uint32_t*)(FW_START_ADDR + (vecIndex << 2));
    }
 
    __HAL_SYSCFG_REMAPMEMORY_SRAM();
 
    __enable_irq();
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  remapMemToSRAM();
  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  Init_UUID();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC_Init();
  MX_CRC_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  
  // Start Timer for ADC flag
  HAL_TIM_Base_Start_IT(&htim1); // UpdateSensorsValues(); 
  HAL_TIM_Base_Start_IT(&htim3); // Update UART Rx IT ();
  
  CircularBuffer_Init(&kq130_buf);
  HAL_TIM_Base_Start_IT(&htim1);  
  HAL_TIM_Base_Start_IT(&htim3);
  
  HAL_GPIO_WritePin(PLC_RESET_GPIO_Port, PLC_RESET_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(PLC_MODE_GPIO_Port, PLC_MODE_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    //Receive 1 byte from KQ130F
    HAL_UART_Receive_IT(&huart1, &plc_uart_buf, 1);
    // if there is a chance to contain full packet plc_circular_buf_data_size
    // must be greater or equal than PACKET_SIZE
    uint16_t len = CircularBuffer_GetLength(&kq130_buf);
    
    if (len >= REGULAR_PACKET_SIZE || len >= MAINTENANCE_PACKET_SIZE) 
    {
      new_byte_received = 0;
      CircularBuffer_GetLastNValues(&kq130_buf, packet_analyze_buf, PACKET_SIZE);
      if (IsValidMaintenancePacket(packet_analyze_buf))
      {
        ProceedMaintenanceCmd(packet_analyze_buf);
        CircularBuffer_RemoveLastNValues(&kq130_buf, PACKET_SIZE); //packet was read and throwed away
      }
      else if (IsValidRegularPacket(packet_analyze_buf))
      {
        ProceedRegularCmd(packet_analyze_buf);
        CircularBuffer_RemoveLastNValues(&kq130_buf, PACKET_SIZE); //packet was read and throwed away
      }
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);

  if(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_0)
  {
  Error_Handler();  
  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {
    
  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_HSI14_Enable();

   /* Wait till HSI14 is ready */
  while(LL_RCC_HSI14_IsReady() != 1)
  {
    
  }
  LL_RCC_HSI14_SetCalibTrimming(16);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {
  
  }
  LL_SetSystemCoreClock(8000000);
  LL_RCC_HSI14_EnableADCControl();
  LL_RCC_SetUSARTClockSource(LL_RCC_USART1_CLKSOURCE_PCLK1);
}

/* USER CODE BEGIN 4 */

/*! \brief Jumps to the bootloader.
 */
void JumpToBootloader(void) 
{
  HAL_TIM_Base_Stop(&htim1);

  if (((*(__IO uint32_t*)BOOT_FLAG_ADDRESS) & 0x2FFE0000 ) == 0x20000000)
  {
    /* First, disable all IRQs */
    __disable_irq();

    /* Get the main application start address */
    uint32_t jump_address = *(__IO uint32_t *)(BOOT_FLAG_ADDRESS + 4);

    /* Set the main stack pointer to to the application start address */
    __set_MSP(*(__IO uint32_t *)BOOT_FLAG_ADDRESS);

    // Create function pointer for the main application
    void (*pmain_app)(void) = (void (*)(void))(jump_address);

    // Now jump to the main application
    pmain_app();
  }
} 

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim3)
	{	

	}
  if (htim == &htim1) 
	{	

	}
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart1)	
  {
    HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
    CircularBuffer_Put_OW(&kq130_buf, plc_uart_buf);
    new_byte_received = 1;
  }
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
