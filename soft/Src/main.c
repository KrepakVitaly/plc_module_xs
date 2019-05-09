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
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "dali_interface_lib.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define PACKET_SIZE 13
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int send;
uint8_t dali_cntr;

//                                SIZE| HEAD OF PACKET |     ADDRESS     | STAT | LEVEL |   CRC32 (not used)  |
//                                  0     1     2    3      4    5     6      7     8     9     10   11    12
uint8_t plc_uart_answer_ok[13] = {0x0c, 0x62, 0xAA, 0x77, 0x00, 0x00, 0x02, 0x00, 0x0a, 0x0c, 0x26, 0x68, 0x68};

uint32_t dali_cmd = 0x01FE01; //0000 0001 - start, 0 000 000 0 0001 0001
uint32_t dali_cmd_sh= 0x01FE01; //0000 0001 - start, 1 111 111 0 0000 0001

uint8_t plc_uart_buf = 0;
uint8_t plc_uart_cycle_buf[PLC_UART_CYCLE_BUF_LEN] = {0};
uint32_t plc_circular_buf_data_size;
uint32_t plc_circular_buf_start;
uint32_t plc_circular_buf_end;
uint8_t new_byte_received = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  plc_circular_buf_start = 0;
  plc_circular_buf_end = 0;
  send = 0;
  dali_cntr = 0;
  
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
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim1);  
  HAL_TIM_Base_Start_IT(&htim3);
  
  HAL_GPIO_WritePin(PLC_RESET_GPIO_Port, PLC_RESET_Pin, GPIO_PIN_SET);
  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
  
  uint32_t plc_circular_buf_clear_size = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
    //Receive 1 byte from KQ130F
    if (new_byte_received == 0)
    {
      HAL_UART_Receive_IT(&huart1, &plc_uart_buf, 1);
      continue;
    }
    
    plc_circular_buf_clear_size = 0;
    
    // if there is a chance to contain full packet plc_circular_buf_data_size
    // must be greater or equal than PACKET_SIZE
    if (plc_circular_buf_data_size >= PACKET_SIZE) 
    {
      // try to find head of packet
      // Byte No0 Contain PACKET_SIZE so start with the 1st byte in circular buf
      for (int i = 1; i <= plc_circular_buf_data_size - PACKET_SIZE + 1; i++) 
      {
        //if packet was found
        if ( plc_uart_cycle_buf[plc_circular_buf_start + i + 1] == 0x56 && // byte No1
             plc_uart_cycle_buf[plc_circular_buf_start + i + 2] == 0x12 && // byte No2
             plc_uart_cycle_buf[plc_circular_buf_start + i + 3] == 0x54  ) // byte No3
        {
          //if address is valid
          if ( plc_uart_cycle_buf[plc_circular_buf_start + i + 4] == 0x00 &&
               plc_uart_cycle_buf[plc_circular_buf_start + i + 5] == 0x00 &&
               plc_uart_cycle_buf[plc_circular_buf_start + i + 6] == 0x01 ) //if address is 0x000001 //bytes No 4-6 
          {
            HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
            dali_cmd = 0x01FE00 + plc_uart_cycle_buf[plc_circular_buf_start + i + 8]; // byte No8
            plc_uart_answer_ok[4] = 0x00;
            plc_uart_answer_ok[5] = 0x00;
            plc_uart_answer_ok[6] = 0x01;
            plc_uart_answer_ok[7] =  0x00; //status ok
            plc_uart_answer_ok[8] =  dali_cmd & 0xFF; //level
            while (HAL_UART_Transmit(&huart1, plc_uart_answer_ok, 13, 100) != HAL_OK);
          }
          plc_circular_buf_clear_size += PACKET_SIZE; //packet was read and throwed away
          break;
        }
         //if broadcast packet was found
        else if ( plc_uart_cycle_buf[plc_circular_buf_start + i + 1] == 0x32 && // byte No1
                  plc_uart_cycle_buf[plc_circular_buf_start + i + 2] == 0x02 && // byte No2
                  plc_uart_cycle_buf[plc_circular_buf_start + i + 3] == 0x77  ) // byte No3
        {
          HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
          dali_cmd = 0x01FE00 + plc_uart_cycle_buf[plc_circular_buf_start + i + 8]; // byte No8
          plc_uart_answer_ok[7] =  0x00; //status ok
          plc_uart_answer_ok[8] =  dali_cmd & 0xFF; //level

          plc_circular_buf_clear_size += PACKET_SIZE; //packet was read and throwed away
          break;
        }
        else //if packet was not found, clear the previous byte
        {
          plc_circular_buf_clear_size++;
        }
      }
      //clear the first N bytes which is not the head of packet and processed packet
      plc_circular_buf_start += plc_circular_buf_clear_size;
      plc_circular_buf_data_size -= plc_circular_buf_clear_size;
    }
    new_byte_received = 0;
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

  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
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
	if (htim == &htim3)
	{	
    send = 1;
	}
  if (htim == &htim1) 
	{	
    if (send == 1)
    {
      if (dali_cntr >= 34)
      {
        dali_cntr = 0;
        HAL_GPIO_WritePin(DALI_TX_GPIO_Port, DALI_TX_Pin, GPIO_PIN_RESET);
        send = 0;
        return;
      }
      
      if (dali_cntr % 2 == 0)
      {
        if( ((dali_cmd >> (17 - dali_cntr/2 - 1)) & 0x01) == 1 )
          HAL_GPIO_WritePin(DALI_TX_GPIO_Port, DALI_TX_Pin, GPIO_PIN_SET);
        else
          HAL_GPIO_WritePin(DALI_TX_GPIO_Port, DALI_TX_Pin, GPIO_PIN_RESET);
      }
      else
      {
        HAL_GPIO_TogglePin(DALI_TX_GPIO_Port, DALI_TX_Pin);
      }
      
      dali_cntr++;
    }
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart1)	
  {
    plc_circular_buf_end+=1;
    
    if (plc_circular_buf_end >= PLC_UART_CYCLE_BUF_LEN)
    {
      plc_circular_buf_end = PLC_UART_CYCLE_BUF_LEN - plc_circular_buf_end;
    }
    
    if (plc_circular_buf_end == plc_circular_buf_start)
    {
      plc_circular_buf_start++;
    }
    
    if (plc_circular_buf_end >= plc_circular_buf_start)
      plc_circular_buf_data_size = plc_circular_buf_end - plc_circular_buf_start;
    else
      plc_circular_buf_data_size = PLC_UART_CYCLE_BUF_LEN - plc_circular_buf_start + plc_circular_buf_end;
    
    plc_uart_cycle_buf[plc_circular_buf_end] = plc_uart_buf;
    
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
