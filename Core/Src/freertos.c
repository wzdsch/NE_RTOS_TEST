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

DJI_Motor_t GM6020;
float GM6020_PID_data[5] = {5.0f, 0.1f, 0.0f, 16000.0f, 5000.0f};

DJI_Motor_t M3508;
float M3508_PID_data[5] = {5.0f, 0.1f, 0.0f, 16000.0f, 5000.0f};
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
  DJI_Motor_TxInitAll();
	
  DJI_Motor_Init(&M3508, &hcan1, DJI_MOTOR_TYPE_M3508, 0x208, NULL, GM6020_I_CTRL);
  MotorCtrl_Init(&M3508.ctrl, MOTOR_CTRL_PID_INTERNAL, MOTOR_CTRL_OUT_PID, 16384, &M3508);
  MotorCtrl_InternalPid_Init(&M3508.ctrl, PID_POSITION, &M3508.measure_data.spd_rpm_f, M3508_PID_data);
  DJI_Motor_Enable(&M3508);
  MotorCtrl_SetTarget(&M3508.ctrl, 100);
	
	DJI_Motor_Init(&GM6020, &hcan1, DJI_MOTOR_TYPE_GM6020, 0x20a, NULL, GM6020_I_CTRL);
	MotorCtrl_Init(&GM6020.ctrl, MOTOR_CTRL_PID_INTERNAL, MOTOR_CTRL_OUT_PID, 16384, &GM6020);
	MotorCtrl_InternalPid_Init(&GM6020.ctrl, PID_POSITION, &GM6020.measure_data.spd_rpm_f, GM6020_PID_data);
	DJI_Motor_Enable(&GM6020);
	MotorCtrl_SetTarget(&GM6020.ctrl, 100);
  while (1)
  {
    MotorCtrl_Calc(&M3508.ctrl);
    DJI_Motor_Transmit(&hcan1, 0x1ff);
		
		MotorCtrl_Calc(&GM6020.ctrl);
    DJI_Motor_Transmit(&hcan1, 0x2fe);
		
		osDelay(1);
  }
  
}
/* USER CODE END Application */

