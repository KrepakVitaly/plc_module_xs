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

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int send;
int plc_rxed;
uint8_t dali_cntr;
uint8_t my_id = 0x000001;

uint8_t plc_uart_example[16] = {0x0c, 0x56, 0x12, 0x54, 0x00, 0x00, 0x01, 0x01, 0x0a, 0x0c, 0x26, 0x68, 0x95, 0, 0, 0};

uint8_t plc_uart_answer_ok[16] = {0x0b, 0x56, 0x12, 0x54, 0x00, 0x00, 0x01, 0x00, 0x0a, 0x0c, 0x26, 0x68};

uint32_t crc_reg = 0;

uint32_t dali_cmd = 0x01FE01; //0000 0001 - start, 0 000 000 0 0001 0001
uint32_t dali_cmd_sh= 0x01FE01; //0000 0001 - start, 1 111 111 0 0000 0001

//uint32_t dali_cmd = 0x01FE01; //0b11111110 0b00000001
uint8_t plc_uart_cycle_buf[PLC_UART_CYCLE_BUF_LEN] = {0};
uint16_t plc_uart_buf_offset;
uint32_t plc_uart_byte_num;

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
//dali_cmd = 0x01FEFE;
  plc_uart_buf_offset = 0;
  send = 0;
  plc_rxed = 0;
  dali_cntr = 0;
  plc_uart_byte_num = 0;
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
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
    HAL_UART_Receive_IT(&huart1, plc_uart_cycle_buf+plc_uart_buf_offset, 13);
    
    if (plc_uart_buf_offset > 18)
    {
      for (int i = plc_uart_buf_offset-1; i > 18; i--)
      {
        if ( plc_uart_cycle_buf[i-14] == 0x54 &&   // byte No3
             plc_uart_cycle_buf[i-15] == 0x12 &&   // byte No2
             plc_uart_cycle_buf[i-16] == 0x56 )    // byte No1
        {
          if ( plc_uart_cycle_buf[i-11] == 0x01 ) //if address is 0x01 //byte No 6
          {
            dali_cmd = 0x01FE00 + plc_uart_cycle_buf[i-9]; // byte No8
            plc_uart_answer_ok[7] =  dali_cmd & 0xFF;
            HAL_UART_Transmit_IT(&huart1, plc_uart_answer_ok, plc_uart_answer_ok[0]);
            HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
          }
          
          for (int j = i - 17, k = 0; k < 13; j++, k++)
          {
            plc_uart_cycle_buf[j] = 0;
          }
          
          break;
        }
      }
    }
    HAL_Delay(10);
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
    //HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
    plc_uart_buf_offset+=13;
    plc_uart_byte_num+=13;
    if (plc_uart_buf_offset >= PLC_UART_CYCLE_BUF_LEN)
    {
      plc_uart_buf_offset = plc_uart_buf_offset-PLC_UART_CYCLE_BUF_LEN;
    }
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
