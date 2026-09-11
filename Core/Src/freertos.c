/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "chassis_control.h"

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
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for ChassisTask */
osThreadId_t ChassisTaskHandle;
const osThreadAttr_t ChassisTask_attributes = {
  .name = "ChassisTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
#ifndef ENABLE_UART_COMMAND_TASK
#define ENABLE_UART_COMMAND_TASK 0
#endif
#if ENABLE_UART_COMMAND_TASK
/* Reserved placeholder; host commands are serviced by ChassisTask. */
osThreadId_t UartCommandTaskHandle;
const osThreadAttr_t UartCommandTask_attributes = {
  .name = "UartCommandTask",
  .stack_size = 768 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
#endif

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartChassisTask(void *argument);
#if ENABLE_UART_COMMAND_TASK
void StartUartCommandTask(void *argument);
#endif

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of ChassisTask */
  ChassisTaskHandle = osThreadNew(StartChassisTask, NULL, &ChassisTask_attributes);

#if ENABLE_UART_COMMAND_TASK
  /* Optional placeholder, disabled in normal builds. */
  UartCommandTaskHandle = osThreadNew(StartUartCommandTask, NULL, &UartCommandTask_attributes);
#endif

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartChassisTask */
/**
* @brief Function implementing the ChassisTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartChassisTask */
void StartChassisTask(void *argument)
{
  /* USER CODE BEGIN StartChassisTask */
  const uint32_t tick_hz = osKernelGetTickFreq();
  if (tick_hz < 200U || tick_hz % 200U != 0U)
  {
    Error_Handler(); /* This kernel tick cannot represent exactly 5 ms. */
  }
  const uint32_t period = tick_hz / 200U;
  uint32_t wake = osKernelGetTickCount();
  /* Absolute deadlines prevent execution time accumulating into the period. */
  for(;;)
  {
    Chassis_Update();
    wake += period;
    if (osDelayUntil(wake) != osOK)
    {
      Chassis_RecordDeadlineMiss();
      wake = osKernelGetTickCount();
      wake += period;
      (void)osDelayUntil(wake);
    }
  }
  /* USER CODE END StartChassisTask */
}

/* USER CODE BEGIN Header_StartUartCommandTask */
/**
* @brief Function implementing the UartCommandTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUartCommandTask */
#if ENABLE_UART_COMMAND_TASK
void StartUartCommandTask(void *argument)
{
  /* USER CODE BEGIN StartUartCommandTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartUartCommandTask */
}
#endif

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

