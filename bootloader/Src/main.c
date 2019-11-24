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

#define TIMEOUT_VALUE               SystemCoreClock/3200
#define START_TIMEOUT_VALUE         SystemCoreClock/1600
#define TX_TIMEOUT_VALUE            1000

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
    ERASE  = 0x43,
    WRITE  = 0x31,
    VERIFY = 0x51,
    JUMP   = 0xA1,
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

  /* Hookup Host and Target                                         */
  /* First send a Maintenance packet. Host should reply with ACK    */
  /* If no valid packet is received within TIMEOUT_VALUE            */
  /* then jump to main application                                  */
  if(HAL_UART_Receive(&huart1, pRxBuffer, MAINTENANCE_PACKET_SIZE, START_TIMEOUT_VALUE) != HAL_OK)
  {
    JumpToApplication();
  }
  // wait for the Maintenance packet with UID of this uC
  if(CheckMaintenance(pRxBuffer) != 1)
  {
    Send_NACK(&huart1);
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
    while(HAL_UART_Receive(&huart1, pRxBuffer, 7, TIMEOUT_VALUE) != HAL_OK);
    
    // Check checksum
    if(CheckChecksum(pRxBuffer, 7) != 1)
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
                              + ((uint32_t)pRxBuffer[3] << 8) + ((uint32_t)pRxBuffer[4] << 0);

    // set the number of bytes to be written
    uint8_t numBytes = pRxBuffer[5];
    
    // Receive the data
    while(HAL_UART_Receive(&huart1, pRxBuffer, numBytes+2, TIMEOUT_VALUE) != HAL_OK);
    
    uint8_t checksum_intel = 0;
    for (uint8_t i = 1; i <= numBytes; i++)
      checksum_intel += pRxBuffer[i];
    
    checksum_intel += numBytes;
    checksum_intel += (uint8_t)((startingAddress >> 8)& 0x000000FF);
    checksum_intel += (uint8_t)(startingAddress & 0x000000FF);
    checksum_intel += pRxBuffer[numBytes+1];
    
   // checksum_intel = ~checksum_intel;
  
    // Check checksum of received data
    if(checksum_intel != 0x00)
    {
      // invalid checksum
      Send_NACK(&huart1);
      //return;
    }
    
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
    
    // Send ACK
    Send_ACK(&huart1);
}


static const uint32_t crc_table[0x100] = {
  0x00000000, 0x04C11DB7, 0x09823B6E, 0x0D4326D9, 0x130476DC, 0x17C56B6B, 0x1A864DB2, 0x1E475005, 0x2608EDB8, 0x22C9F00F, 0x2F8AD6D6, 0x2B4BCB61, 0x350C9B64, 0x31CD86D3, 0x3C8EA00A, 0x384FBDBD, 
  0x4C11DB70, 0x48D0C6C7, 0x4593E01E, 0x4152FDA9, 0x5F15ADAC, 0x5BD4B01B, 0x569796C2, 0x52568B75, 0x6A1936C8, 0x6ED82B7F, 0x639B0DA6, 0x675A1011, 0x791D4014, 0x7DDC5DA3, 0x709F7B7A, 0x745E66CD, 
  0x9823B6E0, 0x9CE2AB57, 0x91A18D8E, 0x95609039, 0x8B27C03C, 0x8FE6DD8B, 0x82A5FB52, 0x8664E6E5, 0xBE2B5B58, 0xBAEA46EF, 0xB7A96036, 0xB3687D81, 0xAD2F2D84, 0xA9EE3033, 0xA4AD16EA, 0xA06C0B5D, 
  0xD4326D90, 0xD0F37027, 0xDDB056FE, 0xD9714B49, 0xC7361B4C, 0xC3F706FB, 0xCEB42022, 0xCA753D95, 0xF23A8028, 0xF6FB9D9F, 0xFBB8BB46, 0xFF79A6F1, 0xE13EF6F4, 0xE5FFEB43, 0xE8BCCD9A, 0xEC7DD02D, 
  0x34867077, 0x30476DC0, 0x3D044B19, 0x39C556AE, 0x278206AB, 0x23431B1C, 0x2E003DC5, 0x2AC12072, 0x128E9DCF, 0x164F8078, 0x1B0CA6A1, 0x1FCDBB16, 0x018AEB13, 0x054BF6A4, 0x0808D07D, 0x0CC9CDCA, 
  0x7897AB07, 0x7C56B6B0, 0x71159069, 0x75D48DDE, 0x6B93DDDB, 0x6F52C06C, 0x6211E6B5, 0x66D0FB02, 0x5E9F46BF, 0x5A5E5B08, 0x571D7DD1, 0x53DC6066, 0x4D9B3063, 0x495A2DD4, 0x44190B0D, 0x40D816BA, 
  0xACA5C697, 0xA864DB20, 0xA527FDF9, 0xA1E6E04E, 0xBFA1B04B, 0xBB60ADFC, 0xB6238B25, 0xB2E29692, 0x8AAD2B2F, 0x8E6C3698, 0x832F1041, 0x87EE0DF6, 0x99A95DF3, 0x9D684044, 0x902B669D, 0x94EA7B2A, 
  0xE0B41DE7, 0xE4750050, 0xE9362689, 0xEDF73B3E, 0xF3B06B3B, 0xF771768C, 0xFA325055, 0xFEF34DE2, 0xC6BCF05F, 0xC27DEDE8, 0xCF3ECB31, 0xCBFFD686, 0xD5B88683, 0xD1799B34, 0xDC3ABDED, 0xD8FBA05A, 
  0x690CE0EE, 0x6DCDFD59, 0x608EDB80, 0x644FC637, 0x7A089632, 0x7EC98B85, 0x738AAD5C, 0x774BB0EB, 0x4F040D56, 0x4BC510E1, 0x46863638, 0x42472B8F, 0x5C007B8A, 0x58C1663D, 0x558240E4, 0x51435D53, 
  0x251D3B9E, 0x21DC2629, 0x2C9F00F0, 0x285E1D47, 0x36194D42, 0x32D850F5, 0x3F9B762C, 0x3B5A6B9B, 0x0315D626, 0x07D4CB91, 0x0A97ED48, 0x0E56F0FF, 0x1011A0FA, 0x14D0BD4D, 0x19939B94, 0x1D528623, 
  0xF12F560E, 0xF5EE4BB9, 0xF8AD6D60, 0xFC6C70D7, 0xE22B20D2, 0xE6EA3D65, 0xEBA91BBC, 0xEF68060B, 0xD727BBB6, 0xD3E6A601, 0xDEA580D8, 0xDA649D6F, 0xC423CD6A, 0xC0E2D0DD, 0xCDA1F604, 0xC960EBB3, 
  0xBD3E8D7E, 0xB9FF90C9, 0xB4BCB610, 0xB07DABA7, 0xAE3AFBA2, 0xAAFBE615, 0xA7B8C0CC, 0xA379DD7B, 0x9B3660C6, 0x9FF77D71, 0x92B45BA8, 0x9675461F, 0x8832161A, 0x8CF30BAD, 0x81B02D74, 0x857130C3, 
  0x5D8A9099, 0x594B8D2E, 0x5408ABF7, 0x50C9B640, 0x4E8EE645, 0x4A4FFBF2, 0x470CDD2B, 0x43CDC09C, 0x7B827D21, 0x7F436096, 0x7200464F, 0x76C15BF8, 0x68860BFD, 0x6C47164A, 0x61043093, 0x65C52D24, 
  0x119B4BE9, 0x155A565E, 0x18197087, 0x1CD86D30, 0x029F3D35, 0x065E2082, 0x0B1D065B, 0x0FDC1BEC, 0x3793A651, 0x3352BBE6, 0x3E119D3F, 0x3AD08088, 0x2497D08D, 0x2056CD3A, 0x2D15EBE3, 0x29D4F654, 
  0xC5A92679, 0xC1683BCE, 0xCC2B1D17, 0xC8EA00A0, 0xD6AD50A5, 0xD26C4D12, 0xDF2F6BCB, 0xDBEE767C, 0xE3A1CBC1, 0xE760D676, 0xEA23F0AF, 0xEEE2ED18, 0xF0A5BD1D, 0xF464A0AA, 0xF9278673, 0xFDE69BC4, 
  0x89B8FD09, 0x8D79E0BE, 0x803AC667, 0x84FBDBD0, 0x9ABC8BD5, 0x9E7D9662, 0x933EB0BB, 0x97FFAD0C, 0xAFB010B1, 0xAB710D06, 0xA6322BDF, 0xA2F33668, 0xBCB4666D, 0xB8757BDA, 0xB5365D03, 0xB1F740B4, 
};

uint32_t CalcCRC(uint8_t * pData, uint32_t DataLength)
{
    uint32_t Checksum = 0xFFFFFFFF;
    for(uint32_t i=0; i < DataLength; i++)
    {
        uint8_t top = (uint8_t)(Checksum >> 24);
        top ^= pData[i];
        Checksum = (Checksum << 8) ^ crc_table[top];
    }
    return Checksum;
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
    // APPLICATION_START_ADDRESS
    // 0x08004E0CU
    uint32_t startingAddress = ((uint32_t)pRxBuffer[1] << 24) + ((uint32_t)pRxBuffer[2] << 16) 
                             + ((uint32_t)pRxBuffer[3] << 8 ) + ((uint32_t)pRxBuffer[4] << 0);
   
    uint32_t endingAddress = ((uint32_t)pRxBuffer[5] << 24) + ((uint32_t) pRxBuffer[6] << 16) 
                           + ((uint32_t)pRxBuffer[7] << 8 ) + ((uint32_t)pRxBuffer[8] << 0 );
    
    uint32_t correct_checksum = ((uint32_t)pRxBuffer[9] << 0) + ((uint32_t)pRxBuffer[10] << 8) 
                              + ((uint32_t)pRxBuffer[11] << 16) + ((uint32_t)pRxBuffer[12] << 24 );
    
    uint8_t * data = (uint8_t *)((__IO uint8_t*) startingAddress);
    uint32_t crcResult;
    for(uint32_t address = startingAddress; address < endingAddress; address += 1)    
    {
        data = (uint8_t *)((__IO uint8_t*) address);
        crcResult = HAL_CRC_Accumulate(&hcrc, (uint32_t*)data, 1);
    }    
    
    HAL_UART_Transmit(&huart1, (uint8_t*) &crcResult, 4, 1000);
    
    //uint32_t crc32_flash = CalcCRC((uint8_t*) startingAddress, endingAddress-startingAddress);
   
    //HAL_UART_Transmit(&huart1, (uint8_t*) &crc32_flash, 4, 1000);
    
    //crc32_flash = crc_32_update((uint8_t*) startingAddress, endingAddress-startingAddress);
   
    //HAL_UART_Transmit(&huart1, (uint8_t*) &crc32_flash, 4, 1000);
    
    if(crcResult == correct_checksum)
    {
        Send_ACK(&huart1);
    }
    else
    {
        Send_NACK(&huart1);
    }
    
    //JumpToApplication();
    return;
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
    crc32 = ((crc32 >> 8) & 0x00FFFFFF) ^ crc32_tbl[( crc32 ^ (*tmp) ) & 0xFF];
    // Increment data pointer and decrement length
    tmp++;
    length--;
  }
  return (crc32 ^ 0xFFFFFFFF); 
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

  if (pBuffer[15] != 0x00 && pBuffer[16] != 0x00 && pBuffer[17] != 0x00)
    return 0;

  uint32_t packet_crc = crc_32_update(pBuffer, MAINTENANCE_PACKET_SIZE-MAINTENANCE_PACKET_CRC_SIZE);
  
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
