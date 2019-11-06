/* USER CODE BEGIN Header */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "circular_buffer.h"
#include "plc_greenlight_protocol.h"
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
TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/**Firmware starts at address different than 0*/
#define FW_START_ADDR 0x08002000U
 
/**Force VectorTable to specific memory position defined in linker*/
volatile uint32_t VectorTable[48] __attribute__((section(".RAMVectorTable")));
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
#define BOOT_FLAG_ADDRESS           0x08000000U
#define APPLICATION_START_ADDRESS   0x08002000U
#define TIMEOUT_VALUE               SystemCoreClock/4

#define ACK     0x06U
#define NACK    0x16U

/*****************************************************************************/
/*                          Private Variables                                */
/*****************************************************************************/
/*! \brief The uart handle
 */
//static UART_HandleTypeDef UartHandle;

/*! \brief Buffer for received messages
 */
//static uint8_t pRxBuffer[32];

typedef enum
{
    ERASE = 0x43,
    WRITE = 0x31,
    CHECK = 0x51,
    JUMP  = 0xA1,
} COMMANDS;

/*****************************************************************************/
/*                     Private Function Prototypes                           */
/*****************************************************************************/
/*! \brief Jumps to the main application.
 */
static void JumpToApplication(void);

/*! \brief Sends an ACKnowledge byte to the host.
 *  
 *  \param  *UartHandle The UART handle
 */
//static void Send_ACK(UART_HandleTypeDef *UartHandle);

/*! \brief Sends an NACKnowledge byte to the host.
 *  
 *  \param  *UartHandle The UART handle
 */
//static void Send_NACK(UART_HandleTypeDef *UartHandle);

/*! \brief Validates the checksum of the message.
 *  Validates the received message through a simple checksum.
 *
 *  Each byte of the message (including the received checksum) is XOR'd with 
 *  the previously calculated checksum value. The result should be 0x00.
 *  Note: The first byte of the message is XOR'd with an initial value of 0xFF
 *  
 *  \param  *pBuffer    The buffer where the message is stored.
 *  \param  len         The length of the message;
 *  \retval uint8_t     The result of the validation. 1 = OK. 0 = FAIL
 */
//static uint8_t CheckChecksum(uint8_t *pBuffer, uint32_t len);

/*! \brief Erase flash function
 */
//static void Erase(void);

/*! \brief Write flash function
 */
//static void Write(void);

/*! \brief Check flashed image
 */
//static void Check(void);


//copy the vector table to SRAM
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
  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    HAL_Delay(1000);
    HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
    
    HAL_Delay(1000);
    HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
    
    HAL_Delay(1000);
    HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
    
    HAL_Delay(1000);
    HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
    
    HAL_Delay(1000);
    HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);  
    
    JumpToApplication();

  }

//    /* Hookup Host and Target                           */
//    /* First send an ACK. Host should reply with ACK    */
//    /* If no valid ACK is received within TIMEOUT_VALUE */
//    /* then jump to main application                    */
//    Send_ACK(&UartHandle);
//    if(HAL_UART_Rx(&UartHandle, pRxBuffer, 2, TIMEOUT_VALUE) == HAL_UART_TIMEOUT)
//    {
//        Send_NACK(&UartHandle);
//        JumpToApplication();
//    }
//    if(CheckChecksum(pRxBuffer, 2) != 1 || pRxBuffer[0] != ACK)
//    {
//        Send_NACK(&UartHandle);
//        JumpToApplication();
//    }
//    
//    /* At this point, hookup communication is complete */
//    /* Wait for commands and execute accordingly       */
//    
//	for(;;)
//	{
//        // wait for a command
//        while(HAL_UART_Rx(&UartHandle, pRxBuffer, 2, TIMEOUT_VALUE) == HAL_UART_TIMEOUT);
//        
//        if(CheckChecksum(pRxBuffer, 2) != 1)
//        {
//            Send_NACK(&UartHandle);
//        }
//        else
//        {
//            switch(pRxBuffer[0])
//            {
//                case ERASE:
//                    Send_ACK(&UartHandle);
//                    Erase();
//                    break;
//                case WRITE:
//                    Send_ACK(&UartHandle);
//                    Write();
//                    break;
//                case CHECK:
//                    Send_ACK(&UartHandle);
//                    Check();
//                    break;
//                case JUMP:
//                    Send_ACK(&UartHandle);
//                    JumpToApplication();
//                    break;
//                default: // Unsupported command
//                    Send_NACK(&UartHandle);
//                    break;
//            }
//        }
//        
//	}
//    
//    for(;;);
//    
	return 0;
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
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {
  
  }
  LL_SetSystemCoreClock(8000000);
  LL_RCC_SetUSARTClockSource(LL_RCC_USART1_CLKSOURCE_PCLK1);
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */
  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */
  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */
  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */
  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, PLC_MODE_Pin|PLC_RESET_Pin|LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : PLC_MODE_Pin PLC_RESET_Pin LED2_Pin */
  GPIO_InitStruct.Pin = PLC_MODE_Pin|PLC_RESET_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LED1_Pin */
  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/*! \brief Jumps to the main application.
 */
static void JumpToApplication(void) 
{
#define APPLICATION_START_ADDRESS   0x08002000U

    if (((*(__IO uint32_t*)APPLICATION_START_ADDRESS) & 0x2FFE0000 ) == 0x20000000)
    {
        /* First, disable all IRQs */
        __disable_irq();

        /* Get the main application start address */
        uint32_t jump_address = *(__IO uint32_t *)(APPLICATION_START_ADDRESS + 4);

        /* Set the main stack pointer to to the application start address */
        __set_MSP(*(__IO uint32_t *)APPLICATION_START_ADDRESS);

        // Create function pointer for the main application
        void (*pmain_app)(void) = (void (*)(void))(jump_address);

        // Now jump to the main application
        pmain_app();
    }
} 

///*! \brief Sends an ACKnowledge byte to the host.
// *  
// *  \param  *UartHandle The UART handle
// */
//static void Send_ACK(UART_HandleTypeDef *handle)
//{
//    uint8_t msg[2] = {ACK, ACK};
//    
//    HAL_UART_Tx(handle, msg, 2);
//}

///*! \brief Sends an NACKnowledge byte to the host.
// *  
// *  \param  *UartHandle The UART handle
// */
//static void Send_NACK(UART_HandleTypeDef *handle)
//{
//    uint8_t msg[2] = {NACK, NACK};
//    
//    HAL_UART_Tx(handle, msg, 2);
//}

///*! \brief Validates the checksum of the message.
// *  Validates the received message through a simple checksum.
// *
// *  Each byte of the message (including the received checksum) is XOR'd with 
// *  the previously calculated checksum value. The result should be 0x00.
// *  Note: The first byte of the message is XOR'd with an initial value of 0xFF
// *  
// *  \param  *pBuffer    The buffer where the message is stored.
// *  \param  len         The length of the message;
// *  \retval uint8_t     The result of the validation. 1 = OK. 0 = FAIL
// */
//static uint8_t CheckChecksum(uint8_t *pBuffer, uint32_t len)
//{
//    uint8_t initial = 0xFF;
//    uint8_t result = 0x7F; /* some random result value */
//    
//    result = initial ^ *pBuffer++;
//    len--;
//    while(len--)
//    {
//        result ^= *pBuffer++;
//    }
//    
//    result ^= 0xFF;
//    
//    if(result == 0x00)
//    {
//        return 1;
//    }
//    else
//    {
//        return 0;
//    }
//}

///*! \brief Erase flash function
// */
//static void Erase(void)
//{
//    Flash_EraseInitTypeDef flashEraseConfig;
//    uint32_t sectorError;
//    
//    // Receive the number of pages to be erased (1 byte)
//    // the initial sector to erase  (1 byte)
//    // and the checksum             (1 byte)
//    while(HAL_UART_Rx(&UartHandle, pRxBuffer, 3, TIMEOUT_VALUE) == HAL_UART_TIMEOUT);
//    // validate checksum
//    if(CheckChecksum(pRxBuffer, 3) != 1)
//    {
//        Send_NACK(&UartHandle);
//        return;
//    }
//    
//    if(pRxBuffer[0] == 0xFF)
//    {
//        // global erase: not supported
//        Send_NACK(&UartHandle);
//    }
//    else
//    {
//        // Sector erase:
//        flashEraseConfig.TypeErase = HAL_FLASH_TYPEERASE_SECTOR;
//        
//        // Set the number of sectors to erase
//        flashEraseConfig.NbSectors = pRxBuffer[0];
//        
//        // Set the initial sector to erase
//        flashEraseConfig.Sector = pRxBuffer[1];
//        
//        // perform erase
//        HAL_Flash_Unlock();
//        HAL_Flash_Erase(&flashEraseConfig, &sectorError);
//        HAL_Flash_Lock();
//        
//        Send_ACK(&UartHandle);
//    }
//}


///*! \brief Write flash function
// */
//static void Write(void)
//{
//    uint8_t numBytes;
//    uint32_t startingAddress = 0;
//    uint8_t i;
//    // Receive the starting address and checksum
//    // Address = 4 bytes
//    // Checksum = 1 byte
//    while(HAL_UART_Rx(&UartHandle, pRxBuffer, 5, TIMEOUT_VALUE) == HAL_UART_TIMEOUT);
//    
//    // Check checksum
//    if(CheckChecksum(pRxBuffer, 5) != 1)
//    {
//        // invalid checksum
//        Send_NACK(&UartHandle);
//        return;
//    }
//    else
//    {
//        Send_ACK(&UartHandle);
//    }
//    
//    // Set the starting address
//    startingAddress = pRxBuffer[0] + (pRxBuffer[1] << 8) 
//                    + (pRxBuffer[2] << 16) + (pRxBuffer[3] << 24);
//    
//    // Receive the number of bytes to be written
//    while(HAL_UART_Rx(&UartHandle, pRxBuffer, 2, TIMEOUT_VALUE) == HAL_UART_TIMEOUT);
//    numBytes = pRxBuffer[0];
//    
//    // Receive the data
//    while(HAL_UART_Rx(&UartHandle, pRxBuffer, numBytes+1, TIMEOUT_VALUE) == HAL_UART_TIMEOUT);
//    
//    // Check checksum of received data
//    if(CheckChecksum(pRxBuffer, 5) != 1)
//    {
//        // invalid checksum
//        Send_NACK(&UartHandle);
//        return;
//    }
//    
//    // valid checksum at this point
//    // Program flash with the data
//    i = 0;
//    HAL_Flash_Unlock();
//    while(numBytes--)
//    {
//        HAL_Flash_Program(FLASH_TYPEPROGRAM_BYTE, startingAddress, pRxBuffer[i]);
//        startingAddress++;
//        i++; 
//    }
//    HAL_Flash_Lock();
//    
//    // Send ACK
//    Send_ACK(&UartHandle);
//}

///*! \brief Check flashed image
// */
//static void Check(void)
//{
//    uint32_t startingAddress = 0;
//    uint32_t endingAddress = 0;
//    uint32_t address;
//    uint32_t *data;
//    uint32_t crcResult;
//    
//    // Receive the starting address and checksum
//    // Address = 4 bytes
//    // Checksum = 1 byte
//    while(HAL_UART_Rx(&UartHandle, pRxBuffer, 5, TIMEOUT_VALUE) == HAL_UART_TIMEOUT);
//    
//    // Check checksum
//    if(CheckChecksum(pRxBuffer, 5) != 1)
//    {
//        // invalid checksum
//        Send_NACK(&UartHandle);
//        return;
//    }
//    else
//    {
//        Send_ACK(&UartHandle);
//    }
//    
//    // Set the starting address
//    startingAddress = pRxBuffer[0] + (pRxBuffer[1] << 8) 
//                    + (pRxBuffer[2] << 16) + (pRxBuffer[3] << 24);
//    
//    // Receive the ending address and checksum
//    // Address = 4 bytes
//    // Checksum = 1 byte
//    while(HAL_UART_Rx(&UartHandle, pRxBuffer, 5, TIMEOUT_VALUE) == HAL_UART_TIMEOUT);
//    
//    // Check checksum
//    if(CheckChecksum(pRxBuffer, 5) != 1)
//    {
//        // invalid checksum
//        Send_NACK(&UartHandle);
//        return;
//    }
//    else
//    {
//        Send_ACK(&UartHandle);
//    }
//    
//    // Set the starting address
//    endingAddress = pRxBuffer[0] + (pRxBuffer[1] << 8) 
//                    + (pRxBuffer[2] << 16) + (pRxBuffer[3] << 24);
//    
//    HAL_RCC_CRC_CLK_ENABLE();
//    data = (uint32_t *)((__IO uint32_t*) startingAddress);
//    for(address = startingAddress; address < endingAddress; address += 4)
//    {
//        data = (uint32_t *)((__IO uint32_t*) address);
//        crcResult = HAL_CRC_Accumulate(data, 1);
//    }
//    
//    HAL_RCC_CRC_CLK_DISABLE();
//    if(crcResult == 0x00)
//    {
//        Send_ACK(&UartHandle);
//    }
//    else
//    {
//        Send_NACK(&UartHandle);
//    }
//    
//    JumpToApplication();
//}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
