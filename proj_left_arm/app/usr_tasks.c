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
#include "robot_ctrl_logic.h"
#include "usr_main.h"
#include "usr_freertos.h"

void ArmTask(void *argument)
{
  {
    ArmInit_t arm_init = {
      .JY_ME01 = {
        .huart = &huart1,
        .ID = 0x00,
        .dir = JY_ME01_DIR_REVERSE,
        .zero_angle = 31.f,
        .init_angle = 12.f
      },

      // ------------- motor ----------------
      .yaw1_8009p_init = {
        .hcan = &hcan2,
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
        .hcan = &hcan2,
        .id = 0x11,
        .mst_id = 0x21,
        .PMAX = 12.5f,
        .VMAX = 45.f,
        .TMAX = 54.f,
        .dir = DM_MOTOR_DIR_NORMAL,
        .zero_offset_rad = -1.25f,
        .MotorRxCallback = NULL,
        .p_owner_moudle = &arm
      },

      .pitch2_3508_init = {
        .hcan = &hcan2,
        .tx_id = DJI_MOTOR_TX_200,
        .rx_id = 0x203,
        .type = DJI_MOTOR_TYPE_M3508,
        .dir = DJI_MOTOR_DIR_REVERSE,
        .zero_offset = 0,
        .MotorRxCallback = NULL,
        .p_owner_moudle = &arm
      },

      .yaw2_4310_init = {
        .hcan = &hcan2,
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
        .hcan = &hcan2,
        .tx_id = DJI_MOTOR_TX_200,
        .rx_id = 0x201,
        .type = DJI_MOTOR_TYPE_M2006,
        .dir = DJI_MOTOR_DIR_NORMAL,
        .zero_offset = 0,
        .MotorRxCallback = NULL,
        .p_owner_moudle = &arm
      },

      .end2_2006_init = {
        .hcan = &hcan2,
        .tx_id = DJI_MOTOR_TX_200,
        .rx_id = 0x202,
        .type = DJI_MOTOR_TYPE_M2006,
        .dir = DJI_MOTOR_DIR_NORMAL,
        .zero_offset = 0,
        .MotorRxCallback = NULL,
        .p_owner_moudle = &arm
      },

      // -------------------- param --------------------
      .yaw1_mit_kp = 20.f,
      .yaw1_mit_kd = 0.6f,
      .yaw1_ctrl_max_out = 0.1f,

      .pitch1_mit_kp = 20.f,
      .pitch1_mit_kd = 0.6f,
      .pitch1_ctrl_max_out = 6.f,

      .yaw2_mit_kp = 2.f,
      .yaw2_mit_kd = 0.1f,
      .yaw2_ctrl_max_out = 0.1f,

      .pid_pitch2_ext = {
        .mode = PID_POSITION,
        .kp = 180.f,
        .ki = 0.f,
        .kd = 0.01f,
        .out_limit = 3000.f,
        .i_out_limit = 500.f
      },
      .pid_pitch2_int = {
        .mode = PID_POSITION,
        .kp = 3.00f,
        .ki = 0.f,
        .kd = 0.0f,
        .out_limit = 10000.f,
        .i_out_limit = 2000.f
      },
      .pitch2_ctrl_max_out = 16384.f,

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
        .kp = 1.f,
        .ki = 0.f,
        .kd = 0.f,
        .out_limit = 800.f,
        .i_out_limit = 1000.f
      },
      .end1_ctrl_max_out = 16384.f,

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
        .out_limit = 800.f,
        .i_out_limit = 1000.f
      },
      .end2_ctrl_max_out = 16384.f,

      // ------------------ limit -------------------------
      .yaw1_min_rad = -2.f,
      .yaw1_max_rad = 2.f,
      
      .pitch1_min_rad = -0.1f,
      .pitch1_max_rad = 3.23f,

      .pitch2_min_deg = 26.f,
      .pitch2_max_deg = 109.f,

      .yaw2_min_rad = -3.f,
      .yaw2_max_rad = 1.07f,

      .end_pitch_min_rad = -1.3f,
      .end_pitch_max_rad = 1.3f
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
    static uint8_t i = 0;
    i++;
    i %= 2;
    if (i) {
      JustFloat(arm.yaw1_8009p.processed_measure.pos_rad, arm.pitch1_8009p.processed_measure.pos_rad, \
                JY_ME01.processed_angle, arm.yaw2_4310.processed_measure.pos_rad, &huart6);
    }
    else {
      JustFloat(arm.real_end_pitch_rad, arm.pitch1_8009p.processed_measure.pos_rad, \
                JY_ME01.processed_angle, arm.yaw2_4310.processed_measure.pos_rad, &huart6);
    }
    osDelayUntil(start_tick + 2);
  }
}

void CanCustomCommTask(void *argument) {
  {
    CAN_CustomComm_Tx_Init_t comm_arm2_ctrl_init = {
      .hcan = &hcan1,
      .start_tx_id = CAN_CUSTOM_COMM_START_ID_ARM_CTRL,
      .IDE = CAN_ID_STD,
      .p_buf = &arm2_ctrl_data,
      .size = sizeof(arm2_ctrl_data),
      .pPackFunc = NULL
    };
    CAN_CustomComm_Tx_Init(&comm_arm2_ctrl, &comm_arm2_ctrl_init);
  }

  {
    CAN_CustomComm_Tx_Init_t comm_chassis_ctrl_init = {
      .hcan = &hcan1,
      .start_tx_id = CAN_CUSTOM_COMM_START_ID_CHASSIS_CTRL,
      .IDE = CAN_ID_STD,
      .p_buf = &chassis_ctrl_data,
      .size = sizeof(chassis_ctrl_data),
      .pPackFunc = NULL
    };
    CAN_CustomComm_Tx_Init(&comm_chassis_ctrl, &comm_chassis_ctrl_init);
  }

  {
    CAN_CustomComm_Tx_Init_t comm_push_rods_ctrl_init = {
      .hcan = &hcan1,
      .start_tx_id = CAN_CUSTOM_COMM_START_ID_PUSH_RODS_CTRL,
      .IDE = CAN_ID_STD,
      .p_buf = &push_rods_ctrl_data,
      .size = sizeof(push_rods_ctrl_data),
      .pPackFunc = NULL
    };
    CAN_CustomComm_Tx_Init(&comm_push_rods_ctrl, &comm_push_rods_ctrl_init);
  }

  {
    CAN_CustomComm_Rx_Init_t comm_arm2_fdb_init = {
      .hcan = &hcan1,
      .IDE = CAN_ID_STD,
      .start_rx_id = CAN_CUSTOM_COMM_START_ID_ARM_FDB_TARGET,
      .p_buf = &arm2_fdb_target,
      .size = sizeof(arm2_fdb_target),
      .pUnpackFunc = NULL
    };
    CAN_CustomComm_Rx_Init(&comm_arm2_fdb_target, &comm_arm2_fdb_init);
  }
  while(1)
  {
    uint32_t start_tick = osKernelGetTickCount();
    // CAN_CustomComm_Tx_PackSend(&comm_arm2_ctrl);
    CAN_CustomComm_Tx_PackSend(&comm_chassis_ctrl);
    CAN_CustomComm_Tx_PackSend(&comm_push_rods_ctrl);
    osDelayUntil(start_tick + 3);
  }
}

void RobotCtrlTask(void *argument) {
  while(1)
  {
    uint32_t start_tick = osKernelGetTickCount();
    CtrlLogic_RC();
    osDelayUntil(start_tick + 5);
  }
}

void BspCan1Task(void *argument) {
  BSP_CAN_TxInstance can_tx_msg = {0};
  while (1) {
    osMessageQueueGet(bspCan1TxMsgQueueHandle, &can_tx_msg, 0, portMAX_DELAY);
    while(HAL_CAN_AddTxMessage(can_tx_msg.p_can_handle, &can_tx_msg.tx_header, can_tx_msg.tx_buf, &can_tx_msg.tx_mailbox) != HAL_OK);
  }
}

void BspCan2Task(void *argument) {
  BSP_CAN_TxInstance can_tx_msg = {0};
  while (1) {
    osMessageQueueGet(bspCan2TxMsgQueueHandle, &can_tx_msg, 0, portMAX_DELAY);
    while(HAL_CAN_AddTxMessage(can_tx_msg.p_can_handle, &can_tx_msg.tx_header, can_tx_msg.tx_buf, &can_tx_msg.tx_mailbox) != HAL_OK);
  }
}
