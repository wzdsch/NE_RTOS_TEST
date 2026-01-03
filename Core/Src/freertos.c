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
#include "DM_Motor.h"
#include "arm.h"
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

float set = 0.f;

// DM_Motor_t J8009P;

DJI_Motor_t M3508;
MotorCtrl_t M3508Ctrl;

Arm_t arm;
float yaw1 = .0f, pitch1 = .0f, yaw2 = .0f, pitch2 = .0f;

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
  for (;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void MotorTask(void *argument)
{
  // // DM
  // DM_Motor_Init(&J8009P, &hcan1, 0x01, 0x21, 12.5f, 45.f, 54.f, NULL);
  // DM_Motor_MIT_SetPD(&J8009P, 160.f, 3.f);
  // // DM_Motor_POS_SPD_SetSpd(&J8009P, 40.f);
  // DM_Motor_Enable(&J8009P);
  // // DM_Motor_SPD_SetSpd(&J8009P, set);
  // while (1)
  // {
  //   // DM_Motor_POS_SPD_SetPos(&J8009P, set_pos);
  //   DM_Motor_MIT_SetPos(&J8009P, set);
  //   DM_Motor_MIT_Send(&J8009P);
  //   // DM_Motor_SPD_SetSpd(&J8009P, 10.f);
  //   // DM_Motor_SPD_Send(&J8009P);
  //   osDelay(1);
  //   pvPortMalloc(256);
  // }

  // DJI
  DJI_Motor_Init_t M3508_init = {.hcan = &hcan1, .rx_id = 0x202, .tx_id = DJI_MOTOR_TX_200, .type = DJI_MOTOR_TYPE_M3508, .MotorRxCallback = NULL, .p_owner_moudle = NULL};
  DJI_Motor_Init(&M3508, &M3508_init);
  DJI_Motor_Enable(&M3508);
  MotorCtrl_Init(&M3508Ctrl, MOTOR_CTRL_PID_INTERNAL, MOTOR_CTRL_OUT_PID, 16384, NULL);

  float M3508_PID_INT[5] = {5.0f, 0.1f, .0f, 16384.0f, 5000.0f};
  MotorCtrl_InternalPid_Init(&M3508Ctrl, PID_POSITION, &M3508.measure.spd_rpm_f, M3508_PID_INT);
	MotorCtrl_Enable(&M3508Ctrl);

  while (1) {
    MotorCtrl_SetTarget(&M3508Ctrl, set);
    MotorCtrl_Calc(&M3508Ctrl);
    DJI_Motor_SetCmd(&M3508, M3508Ctrl.final_out);
    DJI_Motor_Transmit(&hcan1, DJI_MOTOR_TX_200);
    osDelay(1);
  }

  // // Arm
  // ArmInit_t arm_init = {
  //   .yaw1_8009p_init = {
  //     .hcan = &hcan1,
  //     .id = 0x11,
  //     .mst_id = 0x21,
  //     .PMAX = 12.5f,
  //     .VMAX = 45.f,
  //     .TMAX = 54.f,
  //     .MotorRxCallback = NULL,
  //     .p_owner_moudle = &arm
  //   },
  //   .pitch1_8009p_init = {
  //     .hcan = &hcan1,
  //     .id = 0x12,
  //     .mst_id = 0x22,
  //     .PMAX = 12.5f,
  //     .VMAX = 45.f,
  //     .TMAX = 54.f,
  //     .MotorRxCallback = NULL,
  //     .p_owner_moudle = &arm
  //   },
  //   .pitch2_3508_init = {
  //     .hcan = &hcan1,
  //     .tx_id = DJI_MOTOR_TX_200,
  //     .rx_id = 0x202,
  //     .type = DJI_MOTOR_TYPE_M3508,
  //     .MotorRxCallback = NULL,
  //     .p_owner_moudle = &arm
  //   },
  //   .yaw2_4310_init = {
  //     .hcan = &hcan1,
  //     .id = 0x15,
  //     .mst_id = 0x25,
  //     .PMAX = 12.5f,
  //     .VMAX = 30.f,
  //     .TMAX = 10.f,
  //     .MotorRxCallback = NULL,
  //     .p_owner_moudle = &arm
  //   }
  // };
  // Arm_Init(&arm, &arm_init);
	// DM_Motor_MIT_SetPD(&arm.yaw1_8009p, 2.f, .2f);
	// DM_Motor_MIT_SetPD(&arm.pitch1_8009p, 2.f, .2f);
	// DM_Motor_MIT_SetPD(&arm.yaw2_4310, 2.f, .2f);
  // Arm_Enable(&arm);
  // while (1)
  // {
  //   Arm_SetTarget(&arm, ARM_LOAD_NONE, yaw1, pitch1, pitch2, yaw2);
  //   Arm_Calc(&arm);

  //   DJI_Motor_Transmit(&hcan1, DJI_MOTOR_TX_200);
	// 	osDelay(1);
  //   DM_Motor_MIT_Send(&arm.yaw1_8009p);
	// 	osDelay(1);
  //   DM_Motor_MIT_Send(&arm.pitch1_8009p);
	// 	osDelay(1);
  //   DM_Motor_MIT_Send(&arm.yaw2_4310);
  //   osDelay(1);
  // }
}
/* USER CODE END Application */

