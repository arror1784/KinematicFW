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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "HGCodeFunction.h"
#include "StepMotorDriver.h"
#include "PWMStepMotor.h"
#include "usart.h"
#include "neoPixel.h"
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
extern DMA_HandleTypeDef hdma_usart2_rx;
extern STMotorHandle_t STMotorDevices[1];
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
  uint8_t buff[100] = {0};
  HGCodeDataControl_t* temp = 0;
  uint32_t commandCount = 0;

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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  MX_TIM13_Init();
  MX_TIM6_Init();
  MX_TIM11_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Transmit_IT(&huart3,"boot board\r\n",12);

  HAL_TIM_Base_Start_IT(&htim11); //touch timer

  STMotorInitHandler(&STMotorDevices[0],&htim1,TIM_CHANNEL_1,EXTI9_5_IRQn);
  STMotorInitParam(&STMotorDevices[0],STMotorCalcMicroToStep(60000 / 60),STMotorCalcMicroToStep(45000 / 60),STMotorCalcMicroToStep(600000 / 60),1);

  startHGCode(&htim6,&huart2,&hdma_usart2_rx);

  setNeoPixel(neoPixel_P,&htim3,TIM_CHANNEL_1,&hdma_tim3_ch1_trig,10,&htim7);

//  HAL_TIM_Base_Start_IT(&htim7); //blank timer


  for(int i = 0 ; i < neoPixel_P->ledCount; i++){
	  setColor(neoPixel_P,converColorTo32((uint8_t)0,(uint8_t)0,(uint8_t)0),i);
  }
  updateColor(neoPixel_P,0);
//  HAL_TIM_PWM_Start_IT(&htim3,TIM_CHANNEL_1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  STMotorWaitingActivate(&STMotorDevices[0],0);
//	  STMotorSetAccelSpeed(&STMotorDevices[0],5244,1);
//	  STMotorGoMicro(&STMotorDevices[0],100000,1);
//	  STMotorWaitingActivate(&STMotorDevices[0],0);
//	  STMotorGoMicro(&STMotorDevices[0],-10000,0);

	  if(HGCodeCheckDataBuffer() == 1){
			temp = HGCodeGetCommandData();

			sprintf(buff,(char*)"Count: %4d, G: %4d, H: %4d, A: %d, B: %d, C: %d M: %d\r\n",
								commandCount++,temp->HGCodeCommand.G,temp->HGCodeCommand.H,
								temp->HGCodeParameter.A,temp->HGCodeParameter.B,temp->HGCodeParameter.C,temp->HGCodeParameter.M);
			HAL_UART_Transmit_IT(&huart3,(char*)buff,strlen(buff));

			if(temp->HGCodeCommand.G != 0){
				switch(temp->HGCodeCommand.G){
				case 01:
					G01(temp);
					break;
				case 02:
					G02(temp);
					break;
				case 03:
					G03(temp);
					break;
				case 28:
					G28(temp);
					break;
				case 27:
					G27(temp);
				default:
					break;
				}
			}else if(temp->HGCodeCommand.H != 0){
				switch(temp->HGCodeCommand.H){
				case 10:
					H10(temp);
					break;
				case 11:
					H11(temp);
					break;
				case 20:
					H20(temp);
					break;
				case 21:
					H21(temp);
					break;
				case 30:
					H30(temp);
					break;
				case 31:
					H31(temp);
					break;
				case 32:
					H32(temp);
					break;
				case 33:
					H33(temp);
					break;
				case 40:
					H40(temp);
					break;
				case 41:
					H41(temp);
					break;
				case 42:
					H42(temp);
					break;
				case 43:
					H43(temp);
					break;
				case 50:
					H50(temp,neoPixel_P);
					break;
				case 51:
					H51(temp,neoPixel_P);
					break;
				case 100:
					H100(temp);
					break;
				case 60:
					H60(temp);
					break;
				default:
					break;
				}
			}
				memset(temp,0x00,sizeof(HGCodeDataControl_t));
		}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode 
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
