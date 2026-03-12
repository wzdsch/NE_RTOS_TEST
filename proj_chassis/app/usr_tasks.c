#include "usr_tasks.h"

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "DJI_Motor.h"
#include "DM_Motor.h"
#include "can_custom_comm.h"
#include "moudle_logic.h"
#include "push_rod.h"
#include "vofa.h"
#include "mecnum_chassis.h"

extern osThreadId_t canTaskHandle;
extern osMessageQueueId_t canTxMsgQueueHandle;
extern osThreadId_t pushRodTaskHandle;
extern osThreadId_t chassisTaskHandle;

extern Chassis_t chassis;
extern PushRod_t push_rod_f;
extern PushRod_t push_rod_b;

extern CAN_Custom_ChassisCtrlData_t chassis_ctrl_data;   // 底盘控制数据
extern CAN_CustomComm_Rx_t can_custom_chassis_ctrl;      // 自定义can通信底盘控制

extern CAN_Custom_PushRodsCtrlData_t push_rods_ctrl_data; // 推杆控制数据
extern CAN_CustomComm_Rx_t can_custom_push_rods_ctrl;    // 自定义can通信推杆控制

void PushRodTask(void *argument)
{
  {
    // --------------- 前推杆初始化 ---------------
    PushRod_Init_t push_rod_f_init = {
      .motor1_init = {
        .hcan = &hcan2,
        .type = DJI_MOTOR_TYPE_M3508,
        .tx_id = DJI_MOTOR_TX_1FF,
        .rx_id = 0x205,
        .dir = DJI_MOTOR_DIR_NORMAL,
        .zero_offset = 0,
        .MotorRxCallback = NULL,
        .p_owner_moudle = &push_rod_f,
      },
      .motor2_init = {
        .hcan = &hcan2,
        .type = DJI_MOTOR_TYPE_M3508,
        .tx_id = DJI_MOTOR_TX_1FF,
        .rx_id = 0x206,
        .dir = DJI_MOTOR_DIR_REVERSE,
        .zero_offset = 0,
        .MotorRxCallback = NULL,
        .p_owner_moudle = &push_rod_f,
      },
      // 电机1位置环
      .motor1_pos_params = {
        .mode = PID_POSITION,
        .kp = 0.1f, .ki = 0.00f, .kd = 0.f, // 典型位置环参数，需微调
        .out_limit = 8000.0f,    // 速度环的目标限幅
        .i_out_limit = 2000.0f
      },
      // 电机1速度环
      .motor1_spd_params = {
        .mode = PID_POSITION,
        .kp = 1.147f, .ki = 0.0f, .kd = 0.0f, // 典型 M3508 速度环参数
        .out_limit = 16384.f,   // 电流最大值 (M3508: 16384)
        .i_out_limit = 1000.0f
      },
      // 电机2位置环
      .motor2_pos_params = {
        .mode = PID_POSITION,
        .kp = 0.1f, .ki = 0.0f, .kd = 0.0f,
        .out_limit = 8000.0f,
        .i_out_limit = 2000.0f
      },
      // 电机2速度环
      .motor2_spd_params = {
        .mode = PID_POSITION,
        .kp = 1.147f, .ki = 0.f, .kd = 0.0f,
        .out_limit = 16384.0f,
        .i_out_limit = 1000.0f
      },
      .err_pid_params = {
        .mode = PID_POSITION,
        .kp = 0.f,
        .ki = 0.0f,
        .kd = 0.0f,
        .out_limit = 0.0f,
        .i_out_limit = 1000.0f
      },
      .motor1_max_out = 16384.0f,
      .motor2_max_out = 16384.0f,
      .calib_current = -2000,     // 校准电流
      .calib_target_count = 1000, // 校准累计计数
    };
    PushRod_Init(&push_rod_f, &push_rod_f_init);
    PushRod_Enable(&push_rod_f);

    // --------------- 后推杆初始化 ---------------
    PushRod_Init_t push_rod_b_init = {
      .motor1_init = {
        .hcan = &hcan2,
        .type = DJI_MOTOR_TYPE_M3508,
        .tx_id = DJI_MOTOR_TX_1FF,
        .rx_id = 0x207,
        .dir = DJI_MOTOR_DIR_NORMAL,
        .zero_offset = 0,
        .MotorRxCallback = NULL,
        .p_owner_moudle = &push_rod_b,
      },
      .motor2_init = {
        .hcan = &hcan2,
        .type = DJI_MOTOR_TYPE_M3508,
        .tx_id = DJI_MOTOR_TX_1FF,
        .rx_id = 0x208,
        .dir = DJI_MOTOR_DIR_REVERSE,
        .zero_offset = 0,
        .MotorRxCallback = NULL,
        .p_owner_moudle = &push_rod_b,
      },
      // 电机1位置环
      .motor1_pos_params = {
        .mode = PID_POSITION,
        .kp = 0.1f, .ki = 0.00f, .kd = 0.f, // 典型位置环参数，需微调
        .out_limit = 8000.0f,    // 速度环的目标限幅
        .i_out_limit = 2000.0f
      },
      // 电机1速度环
      .motor1_spd_params = {
        .mode = PID_POSITION,
        .kp = 1.147f, .ki = 0.0f, .kd = 0.0f, // 典型 M3508 速度环参数
        .out_limit = 16384.f,   // 电流最大值 (M3508: 16384)
        .i_out_limit = 1000.0f
      },
      // 电机2位置环
      .motor2_pos_params = {
        .mode = PID_POSITION,
        .kp = 0.1f, .ki = 0.0f, .kd = 0.0f,
        .out_limit = 8000.0f,
        .i_out_limit = 2000.0f
      },
      // 电机2速度环
      .motor2_spd_params = {
        .mode = PID_POSITION,
        .kp = 1.147f, .ki = 0.f, .kd = 0.0f,
        .out_limit = 16384.0f,
        .i_out_limit = 1000.0f
      },
      .err_pid_params = {
        .mode = PID_POSITION,
        .kp = 0.f, .ki = 0.0f, .kd = 0.0f,
        .out_limit = 0.0f,
        .i_out_limit = 1000.0f
      },
      .motor1_max_out = 16384.0f,
      .motor2_max_out = 16384.0f,
      .calib_current = -2000,     // 校准电流
      .calib_target_count = 1000, // 校准累计计数
    };
    PushRod_Init(&push_rod_b, &push_rod_b_init);
    PushRod_Enable(&push_rod_b);
  }

  // int64_t tar = 0;
  while (1)
  {
    PushRod_Logic();

    DJI_Motor_GroupTransmit(&hcan2, DJI_MOTOR_TX_1FF);
    osDelay(2);
  }
}

void ChassisTask(void* argument) {
  {
    Chassis_Init_t chassis_init = {
      .motor_rf = {
        .hcan = &hcan2,
        .type = DJI_MOTOR_TYPE_M3508,
        .tx_id = DJI_MOTOR_TX_200,
        .rx_id = 0x201,
        .dir = DJI_MOTOR_DIR_NORMAL,
        .zero_offset = 0,
        .MotorRxCallback = NULL,
        .p_owner_moudle = &chassis
      },
      .pid_motor_rf = {
        .mode = PID_POSITION,
        .kp = 5.0f, .ki = 0.1f, .kd = 0.0f,
        .out_limit = 16384.f,
        .i_out_limit = 3000.0f
      },

      .motor_lf = {
        .hcan = &hcan2,
        .type = DJI_MOTOR_TYPE_M3508,
        .tx_id = DJI_MOTOR_TX_200,
        .rx_id = 0x202,
        .dir = DJI_MOTOR_DIR_NORMAL,
        .zero_offset = 0,
        .MotorRxCallback = NULL,
        .p_owner_moudle = &chassis
      },
      .pid_motor_lf = {
        .mode = PID_POSITION,
        .kp = 5.0f, .ki = 0.1f, .kd = 0.0f,
        .out_limit = 16384.f,
        .i_out_limit = 3000.0f
      },

      .motor_lb = {
        .hcan = &hcan2,
        .type = DJI_MOTOR_TYPE_M3508,
        .tx_id = DJI_MOTOR_TX_200,
        .rx_id = 0x203,
        .dir = DJI_MOTOR_DIR_NORMAL,
        .zero_offset = 0,
        .MotorRxCallback = NULL,
      },
      .pid_motor_lb = {
        .mode = PID_POSITION,
        .kp = 5.0f, .ki = 0.1f, .kd = 0.0f,
        .out_limit = 16384.f,
        .i_out_limit = 3000.0f
      },

      .motor_rb = {
        .hcan = &hcan2,
        .type = DJI_MOTOR_TYPE_M3508,
        .tx_id = DJI_MOTOR_TX_200,
        .rx_id = 0x204,
        .dir = DJI_MOTOR_DIR_NORMAL,
      },
      .pid_motor_rb = {
        .mode = PID_POSITION,
        .kp = 5.0f, .ki = 0.1f, .kd = 0.0f,
        .out_limit = 16384.f,
        .i_out_limit = 3000.0f
      }
    };
    Chassis_Init(&chassis, &chassis_init);
  }
  while (1) {
    uint32_t start_tick = osKernelGetTickCount();
    Chassis_Logic();
    DJI_Motor_GroupTransmit(&hcan2, DJI_MOTOR_TX_200);
    osDelayUntil(start_tick + 2);
  }
}

void CanCustomCommTask(void *argument) {
  while (1) {
    uint32_t strt_tick = osKernelGetTickCount();
    osDelayUntil(strt_tick + 10);
  }
}

void CanTask(void *argument) {
  BSP_CAN_TxInstance can_tx_msg = {0};
  while (1) {
    osMessageQueueGet(canTxMsgQueueHandle, &can_tx_msg, 0, portMAX_DELAY);
    while(BSP_CAN_Transmit(&can_tx_msg) != HAL_OK);
  }
}
