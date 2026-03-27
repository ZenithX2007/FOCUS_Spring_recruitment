/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pid.h"
#include "encoder.h"
#include <stdio.h>
#include <string.h>
#include <math.h>   // fabsf
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
    STOP = 0,
	  START,
    GPIO_DRIVE,
    PWM_DRIVE,
    FORWARD,
    REVERSE,
	  SET_SPEED,
	  UNKNOWN_STATE,
    ERROR_STATE
} MotorState;
  
typedef enum
{
    EVT_ON = 0,
    EVT_OFF,
    EVT_MODE_GPIO,
    EVT_MODE_PWM,
    EVT_DIR_FORWARD,
    EVT_DIR_REVERSE,
    EVT_SPEED,
    EVT_STATUS,
    EVT_ERROR
} MotorEvent;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#define RX_BUFFER_SIZE 50

uint8_t rx_byte;                        
char rx_buffer[RX_BUFFER_SIZE];         
uint8_t rx_index = 0;                    
uint8_t cmd_ready = 0;                   

MotorState current_state = STOP;


extern UART_HandleTypeDef huart1;

PID_Controller pid_rpm; 

static float g_target_rpm=0.0f;
static float last_target =0.0f;

volatile float g_current_pid_output = 0.0f;
volatile uint8_t g_pid_update_flag=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void TT_SetSpeed(int16_t speed)
{
    if (speed > 100) speed = 100;
    if (speed < -100) speed = -100;

    int32_t duty = speed;
	
//	  char buf[32];
//    sprintf(buf, "CCR1: %lu, CCR2: %lu\r\n", 
//            (unsigned long)__HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_1),
//            (unsigned long)__HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_2));
//    HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), 100);

  
    if (speed > 0) {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, duty);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
    }
    else if (speed < 0) {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, -duty);
    }
    else {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM4)
    {
			ENCODER_Update_Count();
			g_pid_update_flag = 1;
    }
	
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1)
    {
        if (rx_byte == '\n')
        {
            rx_buffer[rx_index] = '\0';  
            cmd_ready = 1;              
            rx_index = 0;                
        }
        else if (rx_byte != '\r')
        {
            if (rx_index < RX_BUFFER_SIZE - 1)
            {
                rx_buffer[rx_index++] = rx_byte;
            }
						else
						{
							rx_index = 0;
						}
        }
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }
}
const char* MotorStateToString(MotorState state)
{
    switch(state)
    {
        case STOP:        return "STOP";
        case START:       return "START";
        case GPIO_DRIVE:  return "GPIO_DRIVE";
        case PWM_DRIVE:   return "PWM_DRIVE";
        case FORWARD:     return "FORWARD";
        case REVERSE:     return "REVERSE";
        case SET_SPEED:   return "SET_SPEED";
        case ERROR_STATE: return "ERROR_STATE";
        default:          return "UNKNOWN_STATE";
    }
}
MotorEvent GetEventFromString(char *cmd, int *value)
{
    if (strcmp(cmd, "ON") == 0) return EVT_ON;
    if (strcmp(cmd, "OFF") == 0) return EVT_OFF;
    if (strcmp(cmd, "GPIO") == 0) return EVT_MODE_GPIO;
    if (strcmp(cmd, "PWM") == 0) return EVT_MODE_PWM;
    if (strcmp(cmd, "FORWARD") == 0) return EVT_DIR_FORWARD;
    if (strcmp(cmd, "REVERSE") == 0) return EVT_DIR_REVERSE;
    if (strcmp(cmd, "STATUS") == 0) return EVT_STATUS;

    if (strncmp(cmd, "SPEED", 5) == 0)
    {
        if (sscanf(cmd, "%*s %d", value) == 1)
        {   
            return EVT_SPEED;
        }
        else
        {
            return EVT_ERROR;
        }
    }
    return EVT_ERROR;
}
void Command_Parse(char *cmd)
{
    int value = 0;
    MotorEvent event = GetEventFromString(cmd, &value);
    const char* msg = NULL;
    switch (event)
    {
        case EVT_ON:current_state=START;break;
        case EVT_OFF:current_state=STOP;break;
        case EVT_MODE_GPIO:current_state=GPIO_DRIVE;break;
        case EVT_MODE_PWM:current_state=PWM_DRIVE;break;
        case EVT_DIR_FORWARD:current_state=FORWARD;HAL_GPIO_WritePin(L91105_1_GPIO_Port, L91105_1_Pin,GPIO_PIN_SET);HAL_GPIO_WritePin(L91105_2_GPIO_Port, L91105_2_Pin,GPIO_PIN_RESET);break;
        case EVT_DIR_REVERSE:current_state=REVERSE;HAL_GPIO_WritePin(L91105_2_GPIO_Port, L91105_2_Pin,GPIO_PIN_SET);HAL_GPIO_WritePin(L91105_1_GPIO_Port, L91105_1_Pin,GPIO_PIN_RESET);break;
        case EVT_STATUS:break;
        case EVT_SPEED:current_state=SET_SPEED;
											PID_SetTarget(&pid_rpm, value);
											if (fabsf(value-last_target) > 5.0f) {PID_Reset(&pid_rpm);}
											last_target = value;
											//g_current_pid_output=0;
										
											break;
        case EVT_ERROR:current_state=ERROR_STATE;break;
        default:current_state=UNKNOWN_STATE;break;
    }
		 msg = MotorStateToString(current_state);
		 HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
     HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
}

// 3???
void Encoder_Print_Per_2s(void)
{
    static uint32_t t = 0;
    if(HAL_GetTick() - t >= 2000)
    {
        t = HAL_GetTick();
        char buf[128];
        int32_t total = ENCODER_GetCount();
			  float rpm = ENCODER_GetSpeed_rpm();
			  sprintf(buf, ":%d | :%.2f RPM | :%.1f\r\n", total, rpm ,g_current_pid_output);
        HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), 100);
    }
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
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	
	HAL_TIM_Base_Start_IT(&htim4);
	
	HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
	
	ENCODER_Init();
	
	PID_Init(&pid_rpm,
               20.0f,                
               0.4f,                 
               0.0f,                
               -99.0f,              
               99.0f,                
               20.0f,                
               0.85f,                 
               10.0f,                 
               PID_DERIV_MODE_MEASURE 
      );
	PID_SetTarget(&pid_rpm, g_target_rpm);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//		if (cmd_ready)
//    {
//        cmd_ready = 0;
//        Command_Parse(rx_buffer);
//    }
		
		
		Encoder_Print_Per_2s();
		PID_SetTarget(&pid_rpm, 130);
		if(g_pid_update_flag)
    {
      g_pid_update_flag = 0;
   
			float current_rpm = ENCODER_GetSpeed_rpm();
			float target_rpm  = PID_GetTarget(&pid_rpm); 					  
			
			if (fabsf(target_rpm) < 20.0f)
			{
					PID_Reset(&pid_rpm);                 
					g_current_pid_output = 0.0f;                     
			}                
			else if (fabsf(current_rpm) > 2.5f*fabsf(target_rpm))
			{
					PID_Enable(&pid_rpm, 0);
					g_current_pid_output = 0.0f;
			}
			else
			{
					PID_Enable(&pid_rpm, 1);
					g_current_pid_output =PID_Calculate_Fixed(&pid_rpm, target_rpm, current_rpm);
			}
			TT_SetSpeed(g_current_pid_output);				 
    }
	
//    TT_SetSpeed(70);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
