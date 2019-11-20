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
#define BOOT_START_ADDRESS           0x08000000U
#define APPLICATION_START_ADDRESS   0x08004000U
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
        VectorTable[vecIndex] = *(volatile uint32_t*)(BOOT_START_ADDRESS + (vecIndex << 2));
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
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim1);
  
  uint8_t ret_carr[2] = "\r\n";  
  HAL_UART_Transmit(&huart1, (uint8_t*) &(STM32_UUID_ADDR[0]), 4, 100);
  HAL_UART_Transmit(&huart1, (uint8_t*) &(STM32_UUID_ADDR[1]), 4, 100);
  HAL_UART_Transmit(&huart1, (uint8_t*) &(STM32_UUID_ADDR[2]), 4, 100);
  HAL_UART_Transmit(&huart1, (uint8_t*) ret_carr, 2, 100);
  HAL_UART_Transmit(&huart1, (uint8_t*) ret_carr, 2, 100); 
  
  HAL_Delay(1000);
  
  HAL_UART_Transmit(&huart1, (uint8_t*) &Signature.x, 2, 100);
  HAL_UART_Transmit(&huart1, (uint8_t*) &Signature.y, 2, 100);
  HAL_UART_Transmit(&huart1, (uint8_t*) &Signature.Wafer, 1, 100);
  HAL_UART_Transmit(&huart1, (uint8_t*) &Signature.Lot, 7, 100);
  HAL_UART_Transmit(&huart1, (uint8_t*) ret_carr, 2, 100);
  HAL_UART_Transmit(&huart1, (uint8_t*) ret_carr, 2, 100);
  
  HAL_Delay(5000);
  
  //Check(); 
  while(1) {}
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
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
            JumpToApplication();
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
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {
    
  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSE);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSE)
  {
  
  }
  LL_SetSystemCoreClock(16000000);
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
        EraseInitStruct.NbPages = pRxBuffer[0]; // TODO change to constant
        
        // Set the initial sector to erase
        EraseInitStruct.PageAddress = APPLICATION_START_ADDRESS;//pRxBuffer[1];
        
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
//    while(HAL_UART_Receive(&huart1, pRxBuffer, 5, TIMEOUT_VALUE) != HAL_OK);
//    
//    // Check checksum
//    if(CheckChecksum(pRxBuffer, 5) != 1)
//    {
//        // invalid checksum
//        Send_NACK(&huart1);
//        return;
//    }
//    else
//    {
//        Send_ACK(&huart1);
//    }
//    
//    // Set the starting address
//    startingAddress = pRxBuffer[0] + (pRxBuffer[1] << 8) 
//                    + (pRxBuffer[2] << 16) + (pRxBuffer[3] << 24);
//    
//    startingAddress = APPLICATION_START_ADDRESS;
//    // Receive the ending address and checksum
//    // Address = 4 bytes
//    // Checksum = 1 byte
//    while(HAL_UART_Receive(&huart1, pRxBuffer, 5, TIMEOUT_VALUE) != HAL_OK);
//    
//    // Check checksum
//    if(CheckChecksum(pRxBuffer, 5) != 1)
//    {
//        // invalid checksum
//        Send_NACK(&huart1);
//        return;
//    }
//    else
//    {
//        Send_ACK(&huart1);
//    }
    
    // Set the starting address
    endingAddress = pRxBuffer[0] + (pRxBuffer[1] << 8) 
                    + (pRxBuffer[2] << 16) + (pRxBuffer[3] << 24);
                    
    const uint32_t FLASH_START_ADDRESS = 0x08004000U; // Flash start address
    const uint32_t FLASH_LENGTH = 0x00004000U; // Flash size                
    endingAddress = 0x08004DA3U;
    startingAddress = 0x08004000U;
    
    data = (uint32_t *)((__IO uint32_t*) startingAddress);
    for(address = startingAddress; address < endingAddress; address += 4)    
    {
        data = (uint32_t *)((__IO uint32_t*) address);
        crcResult = HAL_CRC_Accumulate(&hcrc, data, 1);
    }    
    
    HAL_UART_Transmit(&huart1, (uint8_t*) &crcResult, 4, 100);
    
    uint32_t crc32_flash = crc_32_update((uint8_t*) FLASH_START_ADDRESS,FLASH_LENGTH);
    
    HAL_UART_Transmit(&huart1, (uint8_t*) &crc32_flash, 4, 100);
    
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


// Fast (LUT) CRC32 calculation update (one call, length bytes)
uint32_t crc_32_update(uint8_t *data, uint32_t length) 
{
  // 4-Byte table 0xEDB88320 polynomial
  static const uint32_t crc32_tbl[] = {0x00000000,0x77073096,0xEE0E612C,0x990951BA,0x076DC419,0x706AF48F,0xE963A535,0x9E6495A3,
  0x0EDB8832,0x79DCB8A4,0xE0D5E91E,0x97D2D988,0x09B64C2B,0x7EB17CBD,0xE7B82D07,0x90BF1D91,
  0x1DB71064,0x6AB020F2,0xF3B97148,0x84BE41DE,0x1ADAD47D,0x6DDDE4EB,0xF4D4B551,0x83D385C7,
  0x136C9856,0x646BA8C0,0xFD62F97A,0x8A65C9EC,0x14015C4F,0x63066CD9,0xFA0F3D63,0x8D080DF5,
  0x3B6E20C8,0x4C69105E,0xD56041E4,0xA2677172,0x3C03E4D1,0x4B04D447,0xD20D85FD,0xA50AB56B,
  0x35B5A8FA,0x42B2986C,0xDBBBC9D6,0xACBCF940,0x32D86CE3,0x45DF5C75,0xDCD60DCF,0xABD13D59,
  0x26D930AC,0x51DE003A,0xC8D75180,0xBFD06116,0x21B4F4B5,0x56B3C423,0xCFBA9599,0xB8BDA50F,
  0x2802B89E,0x5F058808,0xC60CD9B2,0xB10BE924,0x2F6F7C87,0x58684C11,0xC1611DAB,0xB6662D3D,
  0x76DC4190,0x01DB7106,0x98D220BC,0xEFD5102A,0x71B18589,0x06B6B51F,0x9FBFE4A5,0xE8B8D433,
  0x7807C9A2,0x0F00F934,0x9609A88E,0xE10E9818,0x7F6A0DBB,0x086D3D2D,0x91646C97,0xE6635C01,
  0x6B6B51F4,0x1C6C6162,0x856530D8,0xF262004E,0x6C0695ED,0x1B01A57B,0x8208F4C1,0xF50FC457,
  0x65B0D9C6,0x12B7E950,0x8BBEB8EA,0xFCB9887C,0x62DD1DDF,0x15DA2D49,0x8CD37CF3,0xFBD44C65,
  0x4DB26158,0x3AB551CE,0xA3BC0074,0xD4BB30E2,0x4ADFA541,0x3DD895D7,0xA4D1C46D,0xD3D6F4FB,
  0x4369E96A,0x346ED9FC,0xAD678846,0xDA60B8D0,0x44042D73,0x33031DE5,0xAA0A4C5F,0xDD0D7CC9,
  0x5005713C,0x270241AA,0xBE0B1010,0xC90C2086,0x5768B525,0x206F85B3,0xB966D409,0xCE61E49F,
  0x5EDEF90E,0x29D9C998,0xB0D09822,0xC7D7A8B4,0x59B33D17,0x2EB40D81,0xB7BD5C3B,0xC0BA6CAD,
  0xEDB88320,0x9ABFB3B6,0x03B6E20C,0x74B1D29A,0xEAD54739,0x9DD277AF,0x04DB2615,0x73DC1683,
  0xE3630B12,0x94643B84,0x0D6D6A3E,0x7A6A5AA8,0xE40ECF0B,0x9309FF9D,0x0A00AE27,0x7D079EB1,
  0xF00F9344,0x8708A3D2,0x1E01F268,0x6906C2FE,0xF762575D,0x806567CB,0x196C3671,0x6E6B06E7,
  0xFED41B76,0x89D32BE0,0x10DA7A5A,0x67DD4ACC,0xF9B9DF6F,0x8EBEEFF9,0x17B7BE43,0x60B08ED5,
  0xD6D6A3E8,0xA1D1937E,0x38D8C2C4,0x4FDFF252,0xD1BB67F1,0xA6BC5767,0x3FB506DD,0x48B2364B,
  0xD80D2BDA,0xAF0A1B4C,0x36034AF6,0x41047A60,0xDF60EFC3,0xA867DF55,0x316E8EEF,0x4669BE79,
  0xCB61B38C,0xBC66831A,0x256FD2A0,0x5268E236,0xCC0C7795,0xBB0B4703,0x220216B9,0x5505262F,
  0xC5BA3BBE,0xB2BD0B28,0x2BB45A92,0x5CB36A04,0xC2D7FFA7,0xB5D0CF31,0x2CD99E8B,0x5BDEAE1D,
  0x9B64C2B0,0xEC63F226,0x756AA39C,0x026D930A,0x9C0906A9,0xEB0E363F,0x72076785,0x05005713,
  0x95BF4A82,0xE2B87A14,0x7BB12BAE,0x0CB61B38,0x92D28E9B,0xE5D5BE0D,0x7CDCEFB7,0x0BDBDF21,
  0x86D3D2D4,0xF1D4E242,0x68DDB3F8,0x1FDA836E,0x81BE16CD,0xF6B9265B,0x6FB077E1,0x18B74777,
  0x88085AE6,0xFF0F6A70,0x66063BCA,0x11010B5C,0x8F659EFF,0xF862AE69,0x616BFFD3,0x166CCF45,
  0xA00AE278,0xD70DD2EE,0x4E048354,0x3903B3C2,0xA7672661,0xD06016F7,0x4969474D,0x3E6E77DB,
  0xAED16A4A,0xD9D65ADC,0x40DF0B66,0x37D83BF0,0xA9BCAE53,0xDEBB9EC5,0x47B2CF7F,0x30B5FFE9,
  0xBDBDF21C,0xCABAC28A,0x53B39330,0x24B4A3A6,0xBAD03605,0xCDD70693,0x54DE5729,0x23D967BF,
  0xB3667A2E,0xC4614AB8,0x5D681B02,0x2A6F2B94,0xB40BBE37,0xC30C8EA1,0x5A05DF1B,0x2D02EF8D};
  
  // Allocate variable for sequential crc calculation and initialize with 0xFFFFFFFF
  uint32_t crc32 = 0xFFFFFFFF;
  // Pointer to data for incrementing 
  uint8_t *tmp = (uint8_t*) data;
  
  // Process 8-bits at a time
  while(length > 0) 
  {
    crc32 = (crc32 >> 8) ^ crc32_tbl[( crc32 ^ (*tmp) ) & 0xFF];
    // Increment data pointer and decrement length
    tmp++;
    length--;
  }
  return (crc32 ^ 0xFFFFFFFF); 
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
