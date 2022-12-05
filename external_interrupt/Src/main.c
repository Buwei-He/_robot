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
#include "can.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "remote_control.h"
#include "CAN_receive.h"
#include "bsp_can.h"
#include "bsp_usart.h"
#include <stdio.h>
#include <stdarg.h>
#include "string.h"
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
const RC_ctrl_t *local_rc_ctrl;
int16_t speed;
int16_t s1;
int16_t s2;
int16_t j3;
int16_t cmd_add = 0;     //计数器可加状态（前伸机构不可执行状态）
int16_t cmd_auto = 0;    //后续自动程序不可执行状态
int16_t cmd_minus = 0;	 //出错减数器不可执行状态
int16_t cnt = 0;         //前伸机构计数器
int16_t box_status = 0;  //球盒状态为低
	

void DR16_initialize()
{
	s1 = local_rc_ctrl->rc.s[0];
	s2 = local_rc_ctrl->rc.s[1];
	j3 = local_rc_ctrl->rc.ch[3];
}

void Forward()
{
	DR16_initialize();
	if(s1 == 2)
	{
		if(j3 == -660 & cmd_add == 0 & cnt<4)
		{
			cnt = cnt + 1;
			cmd_add= 1; //计数器不可加状态（前伸机构可执行状态）
		}
		
		if(cmd_add == 1 & j3 == -660)
		{
			if(cnt == 1)
			{
				HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
				//HAL_GPIO_WritePin(AC1_GPIO_Port, AC1_Pin, GPIO_PIN_SET); //第一级气缸伸出
				//HAL_Delay(500);
				cmd_auto = 1; //后续自动程序可执行状态
			}
			if(cnt == 2)
			{
				HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
				//HAL_GPIO_WritePin(AC1_GPIO_Port, AC1_Pin, GPIO_PIN_SET); //第一级气缸伸出
				//HAL_Delay(500);
				//HAL_GPIO_WritePin(AC2_GPIO_Port, AC2_Pin, GPIO_PIN_SET); //第二级气缸伸出
				//HAL_Delay(500);
				cmd_auto = 1; //后续自动程序可执行状态
			}
			if( cnt == 3)
			{
				HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
				//HAL_GPIO_WritePin(AC1_GPIO_Port, AC1_Pin, GPIO_PIN_SET); //第一级气缸伸出
				//HAL_Delay(500);
				//HAL_GPIO_WritePin(AC2_GPIO_Port, AC2_Pin, GPIO_PIN_SET); //第二级气缸伸出
				//HAL_Delay(500);
				//HAL_GPIO_WritePin(AC3_GPIO_Port, AC3_Pin, GPIO_PIN_SET); //第三级气缸伸出
				//HAL_Delay(500);
				cmd_auto = 1; //后续自动程序可执行状态
			}
			if( cnt == 4)
			{
				HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
				//HAL_GPIO_WritePin(AC1_GPIO_Port, AC1_Pin, GPIO_PIN_SET); //第一级气缸伸出
				//HAL_Delay(500);
				//HAL_GPIO_WritePin(AC2_GPIO_Port, AC2_Pin, GPIO_PIN_SET); //第二级气缸伸出
				//HAL_Delay(500);
				//HAL_GPIO_WritePin(AC3_GPIO_Port, AC3_Pin, GPIO_PIN_SET); //第三级气缸伸出
				//HAL_Delay(500);
				//HAL_GPIO_WritePin(AC4_GPIO_Port, AC4_Pin, GPIO_PIN_SET); //第四级气缸伸出
				//HAL_Delay(500);
				cmd_auto = 1; //后续自动程序可执行状态
			}
		}
	}
}


void Backward()
{
	if(cnt == 1)
			{
				HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,GPIO_PIN_RESET);
				//HAL_GPIO_WritePin(AC1_GPIO_Port, AC1_Pin, GPIO_PIN_RESET); //第一级气缸收回
				//HAL_Delay(500);
			}
			if(cnt ==2)
			{
				HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin,GPIO_PIN_RESET);
				//HAL_GPIO_WritePin(AC1_GPIO_Port, AC1_Pin, GPIO_PIN_RESET); //第一级气缸收回
				//HAL_Delay(500);
				//HAL_GPIO_WritePin(AC2_GPIO_Port, AC2_Pin, GPIO_PIN_RESET); //第二级气缸收回
				//HAL_Delay(500);
			}
			if(cnt ==3)
			{
				HAL_GPIO_WritePin(LED_B_GPIO_Port,LED_B_Pin,GPIO_PIN_RESET);
				//HAL_GPIO_WritePin(AC1_GPIO_Port, AC1_Pin, GPIO_PIN_RESET); //第一级气缸收回
				//HAL_Delay(500);
				//HAL_GPIO_WritePin(AC2_GPIO_Port, AC2_Pin, GPIO_PIN_RESET); //第二级气缸收回
				//HAL_Delay(500);
				//HAL_GPIO_WritePin(AC3_GPIO_Port, AC3_Pin, GPIO_PIN_RESET); //第三级气缸收回
				//HAL_Delay(500);
			}
			if(cnt ==4)
			{
				HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin,GPIO_PIN_RESET);
				//HAL_GPIO_WritePin(AC1_GPIO_Port, AC1_Pin, GPIO_PIN_RESET); //第一级气缸收回
				//HAL_Delay(500);
				//HAL_GPIO_WritePin(AC2_GPIO_Port, AC2_Pin, GPIO_PIN_RESET); //第二级气缸收回
				//HAL_Delay(500);
				//HAL_GPIO_WritePin(AC3_GPIO_Port, AC3_Pin, GPIO_PIN_RESET); //第三级气缸收回
				//HAL_Delay(500);
				//HAL_GPIO_WritePin(AC4_GPIO_Port, AC4_Pin, GPIO_PIN_RESET); //第四级气缸收回
				//HAL_Delay(500);
			}
	
}




void Error_Backward()
{
	DR16_initialize();
	if(s2 == 3 & cmd_minus == 0)
	{
		cmd_minus = 1;
	}
	
	if(s2 == 2)
	{
		if(cmd_minus == 1 & cnt > 0)
		{
			cnt = cnt - 1;
			cmd_minus = 0; //出错减数器不可执行状态
			cmd_add = 0;	 //计数器可加状态
			cmd_auto = 0;  //后续程序不可执行状态
		}
	}
}

void Auto()
{
	s1 = local_rc_ctrl->rc.s[0];
	s2 = local_rc_ctrl->rc.s[1];
	j3 = local_rc_ctrl->rc.ch[3];
	
	if(s1 == 2)
	{
		if(j3 == 0 & cmd_auto == 1)
		{
			if(s2 == 2)
			{
				Backward();
			}
			if(s2 == 3)
			{
				HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,GPIO_PIN_SET);
				HAL_GPIO_WritePin(LED_B_GPIO_Port,LED_B_Pin,GPIO_PIN_SET);
				HAL_Delay(1000);
	//			HAL_GPIO_WritePin(AC5_GPIO_Port, AC5_Pin, GPIO_PIN_SET); //夹爪气缸收缩
	//			HAL_Delay(500);
				//3508电机转动120°
				//2006电机开始转动
				//舵机锁住球门，舵机锁住中间盒子
				Backward();
				cmd_add = 0;
				cmd_auto = 0;
				HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED_B_GPIO_Port,LED_B_Pin,GPIO_PIN_RESET);
	//			HAL_GPIO_WritePin(AC5_GPIO_Port, AC5_Pin, GPIO_PIN_RESET); //夹爪气缸张开
	//			HAL_Delay(500);
				//3508转动
			}
		}
	}
}
	
void box_up()
{
	s1 = local_rc_ctrl->rc.s[0];
	s2 = local_rc_ctrl->rc.s[1];
	if(s1== 1 & box_status ==0)
	{
		//6020 up
		box_status = 1;
	}
}

void box_down()
{
	s1 = local_rc_ctrl->rc.s[0];
	s2 = local_rc_ctrl->rc.s[1];
	if(s1== 2 & box_status ==1)
	{
		//6020 down
		box_status = 0;
	}
}

 void usart_printf(const char *fmt,...)
{
    static uint8_t tx_buf[256] = {0};
    static va_list ap;
    static uint16_t len;
    va_start(ap, fmt);

    //return length of string 
    //返回字符串长度
    len = vsprintf((char *)tx_buf, fmt, ap);
    va_end(ap);
    usart1_tx_dma_enable(tx_buf, len);
}


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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  /* USER CODE BEGIN 2 */
	can_filter_init();
	HAL_CAN_RxFifo0MsgPendingCallback(&hcan2);
	HAL_CAN_RxFifo0MsgPendingCallback(&hcan2);
  remote_control_init();
  usart1_tx_dma_init();
  local_rc_ctrl = get_remote_control_point();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
//		HAL_Delay(100);
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
//		HAL_Delay(100);
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
//		HAL_Delay(200);
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
//		HAL_Delay(100);
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
//		HAL_Delay(100);
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
       
		//		usart_printf(
		//"**********\r\n\
		//ch0:%d\r\n\
		//ch1:%d\r\n\
		//ch2:%d\r\n\
		//ch3:%d\r\n\
		//ch4:%d\r\n\
		//s1:%d\r\n\
		//s2:%d\r\n\
		//**********\r\n",
		//            local_rc_ctrl->rc.ch[0], local_rc_ctrl->rc.ch[1], local_rc_ctrl->rc.ch[2], local_rc_ctrl->rc.ch[3],local_rc_ctrl->rc.ch[4],
		//            local_rc_ctrl->rc.s[0], local_rc_ctrl->rc.s[1]
		//            );
		box_down();
		Forward();
		Error_Backward();
		Auto();
		box_up();
		if(s1 == 2)
		{
			speed = local_rc_ctrl->rc.ch[0] * 15;
			CAN_cmd_gimbal(speed,speed,speed,speed);
			CAN_cmd_chassis(speed,speed,speed,speed);
			HAL_Delay(1);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
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
