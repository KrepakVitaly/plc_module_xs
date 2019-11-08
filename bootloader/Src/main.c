/* USER CODE BEGIN Header */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

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

/* USER CODE BEGIN PV */
/**Firmware starts at address different than 0*/
#define FW_START_ADDR 0x08002000U
FLASH_EraseInitTypeDef EraseInitStruct;
 
/**Force VectorTable to specific memory position defined in linker*/
volatile uint32_t VectorTable[48] __attribute__((section(".RAMVectorTable")));
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
#define BOOT_FLAG_ADDRESS           0x08000000U
#define APPLICATION_START_ADDRESS   0x08002000U
#define TIMEOUT_VALUE               SystemCoreClock/4

#define ACK     0x06U
#define NACK    0x16U

/*****************************************************************************/
/*                          Private Variables                                */
/*****************************************************************************/
/*! \brief Buffer for received messages
 */
static uint8_t pRxBuffer[32];

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
static void Send_ACK(UART_HandleTypeDef *UartHandle);

/*! \brief Sends an NACKnowledge byte to the host.
 *  
 *  \param  *UartHandle The UART handle
 */
static void Send_NACK(UART_HandleTypeDef *UartHandle);

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
static uint8_t CheckChecksum(uint8_t *pBuffer, uint32_t len);

/*! \brief Erase flash function
 */
static void Erase(void);

/*! \brief Write flash function
 */
static void Write(void);

/*! \brief Check flashed image
 */
static void Check(void);

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
  HAL_TIM_Base_Start_IT(&htim1);
  
  uint16_t data = 0x6666;
  
  HAL_FLASH_Unlock();
      HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, 0x08001FE0U, data);
  HAL_FLASH_Lock();

  
//  FLASH_OBProgramInitTypeDef FLASH_RDP;
//  HAL_FLASHEx_OBGetConfig(&FLASH_RDP);
//  if (  FLASH_RDP.RDPLevel == OB_RDP_LEVEL_0 )
//  {
//    HAL_FLASH_Unlock();
//    HAL_FLASH_OB_Unlock();
//    FLASH_RDP.RDPLevel = OB_RDP_LEVEL_1;
//    FLASH_RDP.OptionType = OPTIONBYTE_RDP;
//    HAL_FLASHEx_OBErase();
//    HAL_FLASHEx_OBProgram(&FLASH_RDP); 
//    HAL_FLASH_OB_Launch();
//    HAL_FLASH_OB_Lock();
//    HAL_FLASH_Lock(); 
//  }
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//  while (1)
//  {
//    uint8_t buf = 0x66;
//    HAL_UART_Transmit(&huart1, &buf, 1, 1000);
//    //HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
//    HAL_Delay(1000);
//  }
  //while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//    HAL_Delay(1000);
//    HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
//    
//    HAL_Delay(1000);
//    HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
//    
//    HAL_Delay(1000);
//    HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
//    
   // HAL_Delay(1000);
//    HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
//    
    //HAL_Delay(1000);
//    HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);  
//    
    //JumpToApplication();
  }

  /* Hookup Host and Target                           */
  /* First send an ACK. Host should reply with ACK    */
  /* If no valid ACK is received within TIMEOUT_VALUE */
  /* then jump to main application                    */
  Send_ACK(&huart1);
  if(HAL_UART_Receive(&huart1, pRxBuffer, 2, TIMEOUT_VALUE) != HAL_OK)
  {
    Send_NACK(&huart1);
    JumpToApplication();
  }
  if(CheckChecksum(pRxBuffer, 2) != 1 || pRxBuffer[0] != ACK)
  {
    Send_NACK(&huart1);
    JumpToApplication();
  }
    
  /* At this point, hookup communication is complete */
  /* Wait for commands and execute accordingly       */
    
	for(;;)
	{
    // wait for a command
    while(HAL_UART_Receive(&huart1, pRxBuffer, 2, TIMEOUT_VALUE) != HAL_OK);

    if(CheckChecksum(pRxBuffer, 2) != 1)
    {
        Send_NACK(&huart1);
    }
    else
    {
      switch(pRxBuffer[0])
      {
        case ERASE:
            Send_ACK(&huart1);
            Erase();
            break;
        case WRITE:
            Send_ACK(&huart1);
            //Write();
            break;
        case CHECK:
            Send_ACK(&huart1);
            //Check();
            break;
        case JUMP:
            Send_ACK(&huart1);
            //JumpToApplication();
            break;
        default: // Unsupported command
            Send_ACK(&huart1);
            break;
      }
    }
	}
    
  for(;;);
    
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

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim1)
	{	
    HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
	}

}


/*! \brief Jumps to the main application.
 */
static void JumpToApplication(void) 
{
  HAL_TIM_Base_Stop(&htim1);
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

/*! \brief Sends an ACKnowledge byte to the host.
 *  
 *  \param  *UartHandle The UART handle
 */
static void Send_ACK(UART_HandleTypeDef *handle)
{
    uint8_t msg[2] = {ACK, ACK};
    
    HAL_UART_Transmit(handle, msg, 2, TIMEOUT_VALUE); //TODO: set TX_TIMEOUT_VALUE
}

/*! \brief Sends an NACKnowledge byte to the host.
 *  
 *  \param  *UartHandle The UART handle
 */
static void Send_NACK(UART_HandleTypeDef *handle)
{
    uint8_t msg[2] = {NACK, NACK};
    
    HAL_UART_Transmit(handle, msg, 2, TIMEOUT_VALUE); //TODO: set TX_TIMEOUT_VALUE
}

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
static uint8_t CheckChecksum(uint8_t *pBuffer, uint32_t len)
{
    uint8_t initial = 0xFF;
    uint8_t result = 0x7F; /* some random result value */
    
    result = initial ^ *pBuffer++;
    len--;
    while(len--)
    {
        result ^= *pBuffer++;
    }
    
    result ^= 0xFF;
    
    if(result == 0x00)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/*! \brief Erase flash function
 */
static void Erase(void)
{
    //Flash_EraseInitTypeDef flashEraseConfig;
    //uint32_t sectorError;
    
    // Receive the number of pages to be erased (1 byte)
    // the initial sector to erase  (1 byte)
    // and the checksum             (1 byte)
    while(HAL_UART_Receive(&huart1, pRxBuffer, 3, TIMEOUT_VALUE) != HAL_OK);
    // validate checksum
    if(CheckChecksum(pRxBuffer, 3) != 1)
    {
        Send_NACK(&huart1);
        return;
    }
    
    if(pRxBuffer[0] == 0xFF)
    {
        // global erase: not supported
        Send_NACK(&huart1);
    }
    else
    {
        // Sector erase:
        EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
        
        // Set the number of sectors to erase
        EraseInitStruct.NbPages = pRxBuffer[0];
        
        // Set the initial sector to erase
        EraseInitStruct.PageAddress = 0x08002000U;//pRxBuffer[1];
        
        uint32_t SectorError = 0;
      
        // perform erase
        HAL_FLASH_Unlock();
        //__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR); 
        HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError);

        HAL_FLASH_Lock();
        
        Send_ACK(&huart1);
    }
}


/*! \brief Write flash function
 */
static void Write(void)
{
    uint8_t numBytes;
    uint32_t startingAddress = 0;
    uint8_t i;
    // Receive the starting address and checksum
    // Address = 4 bytes
    // Checksum = 1 byte
    while(HAL_UART_Receive(&huart1, pRxBuffer, 5, TIMEOUT_VALUE) != HAL_OK);
    
    // Check checksum
    if(CheckChecksum(pRxBuffer, 5) != 1)
    {
        // invalid checksum
        Send_NACK(&huart1);
        return;
    }
    else
    {
        Send_ACK(&huart1);
    }
    
    // Set the starting address
    startingAddress = pRxBuffer[0] + (pRxBuffer[1] << 8) 
                    + (pRxBuffer[2] << 16) + (pRxBuffer[3] << 24);
    
    startingAddress = APPLICATION_START_ADDRESS;
    // Receive the number of bytes to be written
    while(HAL_UART_Receive(&huart1, pRxBuffer, 2, TIMEOUT_VALUE) != HAL_OK);
    numBytes = pRxBuffer[0];
    
    // Receive the data
    while(HAL_UART_Receive(&huart1, pRxBuffer, numBytes+1, TIMEOUT_VALUE) != HAL_OK);
    
    // Check checksum of received data
    if(CheckChecksum(pRxBuffer, 5) != 1)
    {
        // invalid checksum
        Send_NACK(&huart1);
        return;
    }
    
    // valid checksum at this point
    // Program flash with the data
    i = 0;
    HAL_FLASH_Unlock();
    while(numBytes--)
    {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, startingAddress, pRxBuffer[i] + (pRxBuffer[i+1] << 8));
        startingAddress++;
        i+=2; 
    }
    HAL_FLASH_Lock();
    
    // Send ACK
    Send_ACK(&huart1);
}

/*! \brief Check flashed image
 */
static void Check(void)
{
    uint32_t startingAddress = 0;
    uint32_t endingAddress = 0;
    uint32_t address;
    uint32_t *data;
    uint32_t crcResult;
    
    // Receive the starting address and checksum
    // Address = 4 bytes
    // Checksum = 1 byte
    while(HAL_UART_Receive(&huart1, pRxBuffer, 5, TIMEOUT_VALUE) != HAL_OK);
    
    // Check checksum
    if(CheckChecksum(pRxBuffer, 5) != 1)
    {
        // invalid checksum
        Send_NACK(&huart1);
        return;
    }
    else
    {
        Send_ACK(&huart1);
    }
    
    // Set the starting address
    startingAddress = pRxBuffer[0] + (pRxBuffer[1] << 8) 
                    + (pRxBuffer[2] << 16) + (pRxBuffer[3] << 24);
    
    startingAddress = APPLICATION_START_ADDRESS;
    // Receive the ending address and checksum
    // Address = 4 bytes
    // Checksum = 1 byte
    while(HAL_UART_Receive(&huart1, pRxBuffer, 5, TIMEOUT_VALUE) != HAL_OK);
    
    // Check checksum
    if(CheckChecksum(pRxBuffer, 5) != 1)
    {
        // invalid checksum
        Send_NACK(&huart1);
        return;
    }
    else
    {
        Send_ACK(&huart1);
    }
    
    // Set the starting address
    endingAddress = pRxBuffer[0] + (pRxBuffer[1] << 8) 
                    + (pRxBuffer[2] << 16) + (pRxBuffer[3] << 24);
    
//    HAL_RCC_CRC_CLK_ENABLE();
//    data = (uint32_t *)((__IO uint32_t*) startingAddress);
//    for(address = startingAddress; address < endingAddress; address += 4)
//    {
//        data = (uint32_t *)((__IO uint32_t*) address);
//        crcResult = HAL_CRC_Accumulate(data, 1);
//    }
//    
//    HAL_RCC_CRC_CLK_DISABLE();
    if(crcResult == 0x00)
    {
        Send_ACK(&huart1);
    }
    else
    {
        Send_NACK(&huart1);
    }
    
    JumpToApplication();
}

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
