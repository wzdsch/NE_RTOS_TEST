/*
 * @Author: Jiang Tianhang 1919524828@qq.com
 * @Date: 2026-03-11 21:53:53
 * @LastEditors: Jiang Tianhang 1919524828@qq.com
 * @LastEditTime: 2026-03-13 17:31:02
 * @FilePath: \proj_right_arm\app\usr_tasks.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
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
#include "moudle_logic.h"
#include "push_rod.h"
#include "vofa.h"
#include "mecnum_chassis.h"

extern osThreadId_t armTaskHandle;
extern osThreadId_t canTaskHandle;
extern osMessageQueueId_t canTxMsgQueueHandle;

extern CAN_Custom_ArmCtrlData_t arm_ctrl_data;
extern CAN_CustomComm_Rx_t comm_arm_ctrl;

extern CAN_CustomComm_Tx_t comm_arm_fdb_target;
extern ArmTarget_t arm_fdb_target;

extern Arm_t arm;

void ArmTask(void *argument)
{
  {
    ArmInit_t arm_init = {
      .JY_ME01 = {
        .huart = &huart1,
        .ID = 0x00,
        .dir = JY_ME01_DIR_REVERSE,
        .zero_angle = 15.f,
        .init_angle = 6.f
      },
      .yaw1_8009p_init = {
        .hcan = &hcan1,
        .id = 0x12,
        .mst_id = 0x22,
        .PMAX = 12.5f,
        .VMAX = 45.f,
        .TMAX = 54.f,
        .dir = DM_MOTOR_DIR_REVERSE,
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
        .zero_offset_rad = -1.56f,
        .MotorRxCallback = NULL,
        .p_owner_moudle = &arm
      },
      .pitch2_3508_init = {
        .hcan = &hcan1,
        .tx_id = DJI_MOTOR_TX_200,
        .rx_id = 0x203,
        .type = DJI_MOTOR_TYPE_M3508,
        .dir = DJI_MOTOR_DIR_REVERSE,
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
        .dir = DM_MOTOR_DIR_REVERSE,
        .zero_offset_rad = 1.f,
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
      },
      .pid_pitch2_ext = {
        .mode = PID_POSITION,
        .kp = 166.f,
        .ki = 0.f,
        .kd = 0.01f,
        .out_limit = 3000.f,
        .i_out_limit = 500.f
      },
      .pid_pitch2_int = {
        .mode = PID_POSITION,
        .kp = 2.f,
        .ki = 0.f,
        .kd = 0.0f,
        .out_limit = 5000.f,
        .i_out_limit = 1000.f
      },
      .pid_end1_ext = {
        .mode = PID_POSITION,
        .kp = 0.03f,
        .ki = 0.f,
        .kd = 0.f,
        .out_limit = 15000.f,
        .i_out_limit = 2000.f
      },
      .pid_end1_int = {
        .mode = PID_POSITION,
        .kp = 1.1f,
        .ki = 0.f,
        .kd = 0.f,
        .out_limit = 16384.f,
        .i_out_limit = 1000.f
      },
      .pid_end2_ext = {
        .mode = PID_POSITION,
        .kp = 0.03f,
        .ki = 0.f,
        .kd = 0.f,
        .out_limit = 15000.f,
        .i_out_limit = 2000.f
      },
      .pid_end2_int = {
        .mode = PID_POSITION,
        .kp = 1.f,
        .ki = 0.f,
        .kd = 0.f,
        .out_limit = 16384.f,
        .i_out_limit = 1000.f
      },
      
    };
    Arm_Init(&arm, &arm_init);
  }
  while(1)
  {
    uint32_t start_tick = osKernelGetTickCount();
    ArmLogic();
    Arm_Calc(&arm);

    // 发送
    DM_Motor_MIT_Send(&arm.yaw1_8009p);
    DM_Motor_MIT_Send(&arm.pitch1_8009p);
    DM_Motor_MIT_Send(&arm.yaw2_4310);
    DJI_Motor_GroupTransmit(&hcan2, DJI_MOTOR_TX_200);
    osDelayUntil(start_tick + 2);
  }
}

void CanCustomCommTask(void *argument) {
  {
    CAN_CustomComm_Rx_Init_t comm_arm_ctrl_rx_init = {
      .hcan = &hcan1,
      .start_rx_id = CAN_CUSTOM_COMM_START_ID_ARM_CTRL,
      .IDE = CAN_ID_STD,
      .p_buf = &arm_ctrl_data,
      .size = sizeof(arm_ctrl_data),
      .pUnpackFunc = NULL
    };
    CAN_CustomComm_Rx_Init(&comm_arm_ctrl, &comm_arm_ctrl_rx_init);
  }
  {
    CAN_CustomComm_Tx_Init_t comm_arm_fdb_init = {
      .hcan = &hcan1,
      .start_tx_id = CAN_CUSTOM_COMM_START_ID_ARM_FDB_TARGET,
      .IDE = CAN_ID_STD,
      .p_buf = &arm_fdb_target,
      .size = sizeof(arm_fdb_target),
      .pPackFunc = NULL
    };
    CAN_CustomComm_Tx_Init(&comm_arm_fdb_target, &comm_arm_fdb_init);
  }
  while(1)
  {
    
    osDelay(1);
  }
}

void CanTask(void *argument) {
  BSP_CAN_TxInstance can_tx_msg = {0};
  while (1) {
    osMessageQueueGet(canTxMsgQueueHandle, &can_tx_msg, 0, portMAX_DELAY);
    while(BSP_CAN_Transmit(&can_tx_msg) != HAL_OK);
  }
}
