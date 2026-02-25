#include "usr_tasks.h"

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "DJI_Motor.h"
#include "DM_Motor.h"
#include "arm.h"
#include "remote_receive.h"
#include "can_custom_comm.h"
#include "JY_ME01.h"
#include "robot_logic.h"

extern osThreadId_t armTaskHandle;
extern osThreadId_t canTaskHandle;
extern osMessageQueueId_t canTxMsgQueueHandle;

void ArmTask(void *argument)
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
    ArmLogic_RC();
    osDelayUntil(start_tick + 2);
  }
}

void CanTask(void *argument) {
  BSP_CAN_TxInstance can_tx_msg = {0};
  while (1) {
    osMessageQueueGet(canTxMsgQueueHandle, &can_tx_msg, 0, portMAX_DELAY);
    while(!BSP_CAN_Transmit(&can_tx_msg));
  }
}
