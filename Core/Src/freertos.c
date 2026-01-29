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
#include "remote_receive.h"
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

// DJI_Motor_t M3508;
// MotorCtrl_t M3508Ctrl;

Arm_t arm;
float yaw1 = -0.36f, pitch1 = -0.8f, yaw2 = 1.13f, pitch2 = .0f, end_1 = 0.f, end_2 = 0.f;
float yaw1_step = 0.000005f, pitch1_step = 0.000005f, pitch2_step = 0.2f, yaw2_step = 0.000005f, \
      end_pitch_step = 0.4f, end_yaw_step = 0.8f;

// BSP_CAN_RxInstance can_rx_instance;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
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
void MX_FREERTOS_Init(void)
{
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
  ArmInit_t arm_init = {
    .yaw1_8009p_init = {
      .hcan = &hcan1,
      .id = 0x12,
      .mst_id = 0x22,
      .PMAX = 12.5f,
      .VMAX = 45.f,
      .TMAX = 54.f,
      .dir = DM_MOTOR_DIR_NORMAL,
      .zero_offset_rad = 0.f,
      .MotorRxCallback = NULL,
      .p_owner_moudle = &arm
    },
    .pitch1_8009p_init = {
      .hcan = &hcan1,
      .id = 0x11,
      .mst_id = 0x21,
      .PMAX = 12.5f,
      .VMAX = 45.f,
      .TMAX = 54.f,
      .dir = DM_MOTOR_DIR_NORMAL,
      .zero_offset_rad = 0.f,
      .MotorRxCallback = NULL,
      .p_owner_moudle = &arm
    },
    .pitch2_3508_init = {
      .hcan = &hcan1,
      .tx_id = DJI_MOTOR_TX_200,
      .rx_id = 0x203,
      .type = DJI_MOTOR_TYPE_M3508,
      .dir = DJI_MOTOR_DIR_NORMAL,
      .zero_offset = 0,
      .MotorRxCallback = NULL,
      .p_owner_moudle = &arm
    },
    .yaw2_4310_init = {
      .hcan = &hcan1,
      .id = 0x13,
      .mst_id = 0x23,
      .PMAX = 12.5f,
      .VMAX = 45.f,
      .TMAX = 54.f,
      .dir = DM_MOTOR_DIR_NORMAL,
      .zero_offset_rad = 0.f,
      .MotorRxCallback = NULL,
      .p_owner_moudle = &arm
    },
    .end1_2006_init = {
      .hcan = &hcan1,
      .tx_id = DJI_MOTOR_TX_200,
      .rx_id = 0x201,
      .type = DJI_MOTOR_TYPE_M2006,
      .dir = DJI_MOTOR_DIR_NORMAL,
      .zero_offset = 0,
      .MotorRxCallback = NULL,
      .p_owner_moudle = &arm
    },
    .end2_2006_init = {
      .hcan = &hcan1,
      .tx_id = DJI_MOTOR_TX_200,
      .rx_id = 0x202,
      .type = DJI_MOTOR_TYPE_M2006,
      .dir = DJI_MOTOR_DIR_NORMAL,
      .zero_offset = 0,
      .MotorRxCallback = NULL,
      .p_owner_moudle = &arm
    }
  };
  Arm_Init(&arm, &arm_init);
  while(1)
  {
    uint32_t start_tick = osKernelGetTickCount();
    if (FSI6Data.right_ch2 == FSI6_CHANNEL_MIN) {
      Arm_Disable(&arm);
    } else if (FSI6Data.right_ch2 == FSI6_CHANNEL_MID) { // 使能 + 计算
      Arm_Enable(&arm);
      Arm_SetTarget(&arm, ARM_LOAD_NONE, yaw1, pitch1, pitch2, yaw2, end_1, end_2);
      Arm_Calc(&arm);
    } else if (FSI6Data.right_ch2 == FSI6_CHANNEL_MAX) { // 使能 + 计算 + 发送

      // ------ enable logical ------

      if (FSI6Data.right_ch1 == FSI6_CHANNEL_MIN) {
        yaw1 -= (FSI6Data.left_x - 1024.f) * yaw1_step;
        pitch1 += (FSI6Data.left_y - 1024.f) * pitch1_step;
        pitch2 -= (FSI6Data.right_y - 1024.f) * pitch2_step;
        yaw2 -= (FSI6Data.right_x - 1024.f) * yaw2_step;
      } else if (FSI6Data.right_ch1 == FSI6_CHANNEL_MAX) {
        end_1 += (FSI6Data.right_x - 1024.f) * end_yaw_step;
        end_2 += (FSI6Data.right_x - 1024.f) * end_yaw_step;
        end_1 += (FSI6Data.right_y - 1024.f) * end_pitch_step;
        end_2 -= (FSI6Data.right_y - 1024.f) * end_pitch_step;
      }

      Arm_SetTarget(&arm, ARM_LOAD_NONE, yaw1, pitch1, pitch2, yaw2, end_1, end_2);
      Arm_Calc(&arm);

      // 发送
      DJI_Motor_GroupTransmit(&hcan1, DJI_MOTOR_TX_200);
      DM_Motor_MIT_Send(&arm.yaw1_8009p);
      DM_Motor_MIT_Send(&arm.pitch1_8009p);
      DM_Motor_MIT_Send(&arm.yaw2_4310);
    } else {
      Arm_Disable(&arm);
    }
    osDelayUntil(start_tick + 2);
  }
}

/* USER CODE END Application */
