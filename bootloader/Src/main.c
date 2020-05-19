/* USER CODE BEGIN Header */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "crc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "circular_buffer.h"
#include "plc_greenlight_protocol.h"
#include "stm32_uid.h"
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
FLASH_EraseInitTypeDef EraseInitStruct;
 
/**Force VectorTable to specific memory position defined in linker*/
volatile uint32_t VectorTable[48] __attribute__((section(".RAMVectorTable")));
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
#define BOOT_START_ADDRESS          0x08000000U
#define APPLICATION_START_ADDRESS   0x08004000U
#define APPLICATION_LENGTH          0x00004000U

#define TIMEOUT_VALUE               SystemCoreClock/100
#define START_TIMEOUT_VALUE         SystemCoreClock/3200
#define TX_TIMEOUT_VALUE            3000

#define ACK     0x06U
#define NACK    0x16U

/*****************************************************************************/
/*                          Private Variables                                */
/*****************************************************************************/
/*! \brief Buffer for received messages
 */
static uint8_t pRxBuffer[64];

typedef enum
{
    ERASE  = 0x43,
    WRITE  = 0x31,
    VERIFY = 0x51,
    JUMP   = 0xA1,
    SWITCH = 0xF1,
} COMMANDS;

/*****************************************************************************/
/*                     Private Function Prototypes                           */
/*****************************************************************************/

uint32_t crc_32_update(uint8_t *data, uint32_t length);

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

static uint8_t CheckMaintenance(uint8_t * pBuffer);

/*! \brief Erase flash function
 */
static void Erase(void);

/*! \brief Write flash function
 */
static void Write(void);

/*! \brief Check flashed image
 */
static void Verify(void);

//copy the vector table to SRAM
void remapMemToSRAM( void )
{
    uint32_t vecIndex = 0;
    __disable_irq();
 
    for(vecIndex = 0; vecIndex < 48; vecIndex++){
        VectorTable[vecIndex] = *(volatile uint32_t*)(BOOT_START_ADDRESS + (vecIndex << 2));
    }
 
    __HAL_SYSCFG_REMAPMEMORY_SRAM();
 
    __enable_irq();
}

static void ClearpRxBuffer (void);
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
//#ifdef ENABLE_RDP_LEVEL1
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
//#endif  
  
  ClearpRxBuffer();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */
  Init_UUID();
  
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET); // ONOFF output
  HAL_GPIO_WritePin(PLC_RESET_GPIO_Port, PLC_RESET_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(PLC_MODE_GPIO_Port, PLC_MODE_Pin, GPIO_PIN_RESET);
  
  HAL_TIM_Base_Start_IT(&htim1);
  /* Hookup Host and Target                                         */
  /* First send a Maintenance packet. Host should reply with ACK    */
  /* If no valid packet is received within TIMEOUT_VALUE            */
  /* then jump to main application                                  */
  
  // wait for the Maintenance packet with UID of this uC
  if(HAL_UART_Receive(&huart1, pRxBuffer, MAINTENANCE_PACKET_SIZE, START_TIMEOUT_VALUE) != HAL_OK)
  {
    JumpToApplication();
  }
  // if no valid packet, silently switch to the app
  if(CheckMaintenance(pRxBuffer) != 1)
  {
    JumpToApplication();
  }
  else
  {
    Send_ACK(&huart1);
  }

  /* At this point, hookup communication is complete */
  /* Wait for commands and execute accordingly       */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // wait for a command
    while(HAL_UART_Receive(&huart1, pRxBuffer, 3, TIMEOUT_VALUE) != HAL_OK);

    if(CheckChecksum(pRxBuffer, 3) != 1)
    {
        Send_NACK(&huart1);
    }
    else
    {
      switch(pRxBuffer[1])
      {
        case ERASE:
            Send_ACK(&huart1);
            Erase();
            break;
        case WRITE:
            Send_ACK(&huart1);
            Write();
            break;
        case VERIFY:
            Send_ACK(&huart1);
            Verify();
            break;
        case JUMP:
            Send_ACK(&huart1);
            JumpToApplication();
            break;
        case SWITCH:
            HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1); // ONOFF output
            Send_ACK(&huart1);
            JumpToApplication();
            break;
        default: // Unsupported command
            Send_NACK(&huart1);
            break;
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  uint8_t msg[3] = {0x02, ACK, 0x02 ^ ACK};
  HAL_UART_Transmit(handle, msg, 3, TX_TIMEOUT_VALUE);
}

/*! \brief Sends an NACKnowledge byte to the host.
 *  
 *  \param  *UartHandle The UART handle
 */
static void Send_NACK(UART_HandleTypeDef *handle)
{
  uint8_t msg[3] = {0x02, NACK, 0x02 ^ NACK};
  HAL_UART_Transmit(handle, msg, 3, TX_TIMEOUT_VALUE);
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
    // 1 Receive the number of pages to be erased (1 byte)
    // 2 the initial sector to erase  (1 byte)
    // 3 and the checksum             (1 byte)
    ClearpRxBuffer();
    while(HAL_UART_Receive(&huart1, pRxBuffer, 4, TIMEOUT_VALUE) != HAL_OK);
    // validate checksum
    if(CheckChecksum(pRxBuffer, 4) != 1)
    {
        Send_NACK(&huart1);
        return;
    }

    if(pRxBuffer[1] == 0xFF)
    {
        // global erase: not supported
        Send_NACK(&huart1);
    }
    else
    {
        // Sector erase:
        EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
        
        // Set the number of sectors to erase
        EraseInitStruct.NbPages = pRxBuffer[1]; // TODO change to constant
        
        // Set the initial sector to erase
        EraseInitStruct.PageAddress = APPLICATION_START_ADDRESS;//pRxBuffer[2];
        
        // contains the configuration information on faulty page in case of error
        // (0xFFFFFFFF means that all the pages have been correctly erased)
        uint32_t PageError = 0; 
  
        // perform erase
        HAL_FLASH_Unlock();
        //__HAL_FLASH_CLEAR_FLAG(FLASH_SR_EOP | FLASH_SR_WRPERR | FLASH_SR_PGERR); 
        HAL_FLASHEx_Erase(&EraseInitStruct, &PageError); // TODO check HAL_OK output
        HAL_FLASH_Lock();
        
        Send_ACK(&huart1); // Erasing finished successfully
    }
}


/*! \brief Write flash function
 */
static void Write(void)
{
    // Update Address START command
    // Receive the starting address, the number of bytes to be written and checksum 
    // 1-4Address = 4 bytes
    // 5numBytes = 1 byte number of bytes to be written
    // 6Checksum = 1 byte
    ClearpRxBuffer();
    while(HAL_UART_Receive(&huart1, pRxBuffer, 7, TIMEOUT_VALUE) != HAL_OK);
    
    // Check checksum
    if(CheckChecksum(pRxBuffer, 7) != 1)
    {
      // invalid checksum
      Send_NACK(&huart1);
      //return;
    }
    else
    {
      Send_ACK(&huart1);
    }
    
    // Set the starting address
    uint32_t startingAddress = ((uint32_t)pRxBuffer[1] << 24) + ((uint32_t)pRxBuffer[2] << 16) 
                              + ((uint32_t)pRxBuffer[3] << 8) + ((uint32_t)pRxBuffer[4] << 0);

    // set the number of bytes to be written
    uint8_t numBytes = pRxBuffer[5];
    
    // Receive the data
    ClearpRxBuffer();
    
    uint8_t rxed_bytes = 0;
    while(rxed_bytes != numBytes+2)
    {
      HAL_UART_Receive(&huart1, pRxBuffer+rxed_bytes, 1, TIMEOUT_VALUE);
      rxed_bytes++;
    };
    HAL_UART_Abort(&huart1);
    
    uint8_t checksum_intel = 0;
    for (uint8_t i = 1; i <= numBytes; i++)
      checksum_intel += pRxBuffer[i];
    
    checksum_intel += numBytes;
    checksum_intel += (uint8_t)((startingAddress >> 8)& 0x000000FF);
    checksum_intel += (uint8_t)(startingAddress & 0x000000FF);
    checksum_intel += pRxBuffer[numBytes+1];

    // valid checksum at this point
    // Program flash with the data
    uint8_t i = 1;
    HAL_FLASH_Unlock();
    while(numBytes)
    {
      HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, startingAddress, pRxBuffer[i] + (pRxBuffer[i+1] << 8));
      startingAddress += 2;
      i += 2;
      numBytes -= 2;
    }
    HAL_FLASH_Lock();
    
    // Check checksum of received data
    if(checksum_intel != 0x00)
    {
      // invalid checksum
      Send_NACK(&huart1);
      //return;
    }
    else
    {
      // Send ACK
      Send_ACK(&huart1);
    }
}


/*! \brief Verify flashed image checksum
 */
static void Verify(void)
{
    // Receive the starting address and checksum
    // AddressStart = 4 bytes
    // AddressEnd = 4 bytes
    // Checksum = 4 byte
    // packet Checksum = 1 byte
    ClearpRxBuffer();
    while(HAL_UART_Receive(&huart1, pRxBuffer, 14, TIMEOUT_VALUE) != HAL_OK);
    
    // Check checksum
    if(CheckChecksum(pRxBuffer, 14) != 1)
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
    uint32_t startingAddress = ((uint32_t)pRxBuffer[1] << 24) + ((uint32_t)pRxBuffer[2] << 16) 
                             + ((uint32_t)pRxBuffer[3] << 8 ) + ((uint32_t)pRxBuffer[4] << 0);
   
    uint32_t endingAddress = ((uint32_t)pRxBuffer[5] << 24) + ((uint32_t) pRxBuffer[6] << 16) 
                           + ((uint32_t)pRxBuffer[7] << 8 ) + ((uint32_t)pRxBuffer[8] << 0 );
    
    uint32_t correct_checksum = ((uint32_t)pRxBuffer[9]  << 0 ) + ((uint32_t)pRxBuffer[10] << 8) 
                              + ((uint32_t)pRxBuffer[11] << 16) + ((uint32_t)pRxBuffer[12] << 24 );
    
    uint8_t * data = (uint8_t *)((__IO uint8_t*) startingAddress);
    
    /* Reset CRC Calculation Unit (hcrc->Instance->INIT is 
    *  written in hcrc->Instance->DR) */
    __HAL_CRC_DR_RESET(&hcrc);
    uint32_t crcResult;
    for(uint32_t address = startingAddress; address < endingAddress; address += 1)    
    {
        data = (uint8_t *)((__IO uint8_t*) address);
        crcResult = HAL_CRC_Accumulate(&hcrc, (uint32_t*)data, 1);
    }    
    
    uint8_t plc_buf[5] = {0x04, ((uint8_t*) &crcResult)[0],
                                ((uint8_t*) &crcResult)[1], 
                                ((uint8_t*) &crcResult)[2], 
                                ((uint8_t*) &crcResult)[3] };
    
    HAL_UART_Transmit(&huart1, plc_buf, 5, TX_TIMEOUT_VALUE);
    
    if(crcResult == correct_checksum)
    {
        Signature.Firmware_CRC32 = crcResult;
        
        HAL_FLASH_Unlock();
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 
                         (FLASH_ADDR_FOR_STORING+sizeof(uint32_t)*FW_CRC_OFFSET), 
                         (uint32_t)crcResult);
        HAL_FLASH_Lock();
        Send_ACK(&huart1);
    }
    else
    {
        Send_NACK(&huart1);
    }
    
    return;
}

static void ClearpRxBuffer (void)
{
  for (uint8_t i = 0; i < 64; i++)
  {
    pRxBuffer[i] = 0;
  }
}

static uint8_t CheckMaintenance(uint8_t * pBuffer)
{
  if (pBuffer[0] != MAINTENANCE_PACKET_SIZE - 1) //check first byte KQ330 protocol
    return 0; //error

  if (pBuffer[1] != MAINTENANCE_PACKET_HEAD_BYTE_1)
    return 0; //error

  if (pBuffer[2] != MAINTENANCE_PACKET_HEAD_BYTE_2)
    return 0; //error
  
  uint32_t rxed_id_part1 = (pBuffer[3] << 0)
                    + (pBuffer[4]  << 8 ) 
                    + (pBuffer[5]  << 16) 
                    + (pBuffer[6]  << 24); 
  uint32_t rxed_id_part2 = (pBuffer[7] << 0)
                    + (pBuffer[8]  << 8 ) 
                    + (pBuffer[9]  << 16) 
                    + (pBuffer[10] << 24); 
  uint32_t rxed_id_part3 = (pBuffer[11] << 0)
                    + (pBuffer[12] << 8 ) 
                    + (pBuffer[13] << 16) 
                    + (pBuffer[14] << 24); 
  
  if (rxed_id_part1 != Signature.idPart1)
    return 0;
  if (rxed_id_part2 != Signature.idPart2)
    return 0;
  if (rxed_id_part3 != Signature.idPart3)
    return 0;

  if (pBuffer[15] != 0x05 && pBuffer[16] != 0x00 && pBuffer[17] != 0x00)
    return 0;

  uint32_t packet_crc = HAL_CRC_Calculate(&hcrc, 
                        (uint32_t*)pBuffer, 
                        MAINTENANCE_PACKET_SIZE-MAINTENANCE_PACKET_CRC_SIZE);
  
  uint32_t rxed_crc = (pBuffer[MAINTENANCE_PACKET_SIZE-4] << 24)
                    + (pBuffer[MAINTENANCE_PACKET_SIZE-3] << 16) 
                    + (pBuffer[MAINTENANCE_PACKET_SIZE-2] << 8) 
                    + (pBuffer[MAINTENANCE_PACKET_SIZE-1] << 0);
  if (rxed_crc != packet_crc)
    return 0;  

  return 1;
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
