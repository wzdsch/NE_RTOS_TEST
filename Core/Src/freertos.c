/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "DJI_Motor.h"
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
osThreadId_t motorTaskHandle;
DJI_Motor_t M3508;
DJI_Motor_t GM6020;
int64_t set_total_pos = 81920;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void MotorTask(void *argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  motorTaskHandle = osThreadNew(MotorTask, NULL, NULL);
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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void MotorTask(void *argument) {
  fp32 none_pid_data[5] = {0};
	fp32 pos_pid_data[5] = {1.0f, 0.0f, 0.0f, 600.0f, 50.0f};
  fp32 spd_pid_data[5] = {5.0f, 0.0f, 0.0f, 16384.0f, 5000.0f};
  DJI_Motor_TxInitAll();
//  DJI_Motor_Init(&M3508, &hcan1, DJI_MOTOR_TYPE_M3508, 0x205, NULL, GM6020_I_CTRL);
//  MotorCommon_Init(&M3508.common_data, MOTOR_CTRL_LOOP_SPD, MOTOR_COMMON_OUT_PID, 16384);
//  MotorCommon_Pid_Init(&M3508.common_data, none_pid_data, spd_pid_data, none_pid_data);
//  MotorCommon_Ctrl_Enable(&M3508.common_data.ctrl_data);
    DJI_Motor_Init(&GM6020, &hcan1, DJI_MOTOR_TYPE_GM6020, 0x20a, NULL, GM6020_I_CTRL);
	MotorCommon_Init(&GM6020.common_data, MOTOR_CTRL_LOOP_TOTALPOS_SPD, MOTOR_COMMON_OUT_PID, 16384);
	MotorCommon_Pid_Init(&GM6020.common_data, pos_pid_data, spd_pid_data, none_pid_data);
  MotorCommon_Ctrl_Enable(&GM6020.common_data.ctrl_data);
  while (1) {
    MotorCommon_Ctrl_SetTarget(&GM6020.common_data, set_total_pos);
    MotorCommon_Calc(&GM6020.common_data);
    DJI_Motor_UpdateTxBuf(&GM6020);
    DJI_Motor_SendGrouping(&hcan1, 0x2fe);
    osDelay(2);
  }
}
/* USER CODE END Application */

