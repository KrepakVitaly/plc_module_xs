/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "crc.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "circular_buffer.h"
#include "plc_mmrpi.h"
#include "stm32_uid.h"
#include "dali_interface_lib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CRC_LENGTH 32
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */




/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int send;
CircularBuffer_Typedef kq130_buf;
uint8_t packet_analyze_buf[PACKET_SIZE];
uint8_t new_byte_received = 0;

uint8_t last_command = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint16_t dist (uint16_t head, uint16_t tail, uint16_t module)
{
  //1. normal mode T > H and |T - H| < MAX_SIZE, dist = T - H
  if (tail > head)
    return tail - head;
  //2. reverse mode T < H and |T - H| < MAX_SIZE, dist = MODULE - (H - T)
  else if (tail < head)
    return module - (head - tail); 
  
  //3. neigbor mode T > H and |E - H| = 1, size = T - H = 1
  //4. neigbor reverse mode H > T and |T - S| = 1, size = MODULE - H + T
  //5. edge reverse mode T > H and |T - H| = 1, size = H + 1 = 1
  //6. max mode T > H and |T - H| = MAX_SIZE
  return 0;
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

  send = 0;
  dali_cntr = 0;
  CircularBuffer_Init(&kq130_buf);
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
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_CRC_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim1);  
  HAL_TIM_Base_Start_IT(&htim3);
  
  HAL_GPIO_WritePin(PLC_RESET_GPIO_Port, PLC_RESET_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(PLC_RESET_GPIO_Port, PLC_RESET_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
    //Receive 1 byte from KQ130F
    //if (new_byte_received == 0)
    {
      HAL_UART_Receive_IT(&huart1, &plc_uart_buf, 1);
      //continue;
    }
    
    // if there is a chance to contain full packet plc_circular_buf_data_size
    // must be greater or equal than PACKET_SIZE
    uint16_t len = CircularBuffer_GetLength(&kq130_buf);
    
    if (new_byte_received && len >= PACKET_SIZE) 
    {
      new_byte_received = 0;
      CircularBuffer_GetLastNValues(&kq130_buf, packet_analyze_buf, PACKET_SIZE);
      //for (int i = 0; i < len - PACKET_SIZE + 1; i++) 
      {
        //for (uint8_t j = 0; j < PACKET_SIZE; j++)
        //  packet_analyze_buf[j] = CircularBuffer_GetRandValue(&kq130_buf, j); 
        
        // try to find head of packet
        //if packet was found
        if ( packet_analyze_buf[1] == 0x56 && // byte No1
             packet_analyze_buf[2] == 0x12 && // byte No2
             packet_analyze_buf[3] == 0x54  ) // byte No3
        {
          //if address is valid
          if ( packet_analyze_buf[4] == MY_ADDR_0 && //bytes No 4-6 
               packet_analyze_buf[5] == MY_ADDR_1 &&
               packet_analyze_buf[6] == MY_ADDR_2 )
          {
            //if set brightness command
            if (packet_analyze_buf[7] == 0x01) // byte No7
              dali_cmd = 0x01FE00 + packet_analyze_buf[8]; // byte No8
            
            plc_uart_answer_ok[4] = MY_ADDR_0;
            plc_uart_answer_ok[5] = MY_ADDR_1;
            plc_uart_answer_ok[6] = MY_ADDR_2;
            plc_uart_answer_ok[7] =  0x01; //status ok
            plc_uart_answer_ok[8] =  dali_cmd & 0xFF; //level
            HAL_UART_Transmit(&huart1, plc_uart_answer_ok, PACKET_SIZE, 10);
            CircularBuffer_RemoveLastNValues(&kq130_buf, PACKET_SIZE); //packet was read and throwed away
            //break;
          }
        }
        //TODO: add byte 0x0D which shows the end of packet and silence time of 2s
        //if broadcast packet was found
        else if ( packet_analyze_buf[1] == 0x32 && // byte No1
                  packet_analyze_buf[2] == 0x02 && // byte No2
                  packet_analyze_buf[3] == 0x77  ) // byte No3
        {
          dali_cmd = 0x01FE00 + packet_analyze_buf[8]; // byte No8
          CircularBuffer_RemoveLastNValues(&kq130_buf, PACKET_SIZE); //packet was read and throwed away
          //break;
        }
        //if multicast packet was found
        else if ( packet_analyze_buf[1] == 0x19 && // byte No1
                  packet_analyze_buf[2] == 0xB3 && // byte No2
                  packet_analyze_buf[3] == 0xEE  ) // byte No3
        {
          uint32_t packet_address = (packet_analyze_buf[4] << 16) + //bytes No 4-6 
                                    (packet_analyze_buf[5] << 8) + 
                                     packet_analyze_buf[6];
          
          uint16_t offset = packet_analyze_buf[7];  // byte No7
          
          if ( my_address >= packet_address && my_address <= (packet_address + offset))
            dali_cmd = 0x01FE00 + packet_analyze_buf[8]; // byte No8
          
          CircularBuffer_RemoveLastNValues(&kq130_buf, PACKET_SIZE); //packet was read and throwed away
          //break;
        }
        else //if packet was not found, clear the previous byte
        {
          //CircularBuffer_RemoveFirstValue(&kq130_buf);
        }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_RTC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim3)
	{	
    send = 1; //start send DALI cmd
	}
  if (htim == &htim1) 
	{	
    if (send == 1)
    {
      send = step_DALI_set_brightness(&dali_cntr); //if cmd was sent, send = 0
    }
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart1)	
  {
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
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
