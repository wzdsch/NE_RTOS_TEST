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
#include "push_rod.h"
#include "vofa.h"
#include "mecnum_chassis.h"

extern osThreadId_t armTaskHandle;
extern osThreadId_t canTaskHandle;
extern osMessageQueueId_t canTxMsgQueueHandle;

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
      }
    };
    Arm_Init(&arm, &arm_init);
  }
  while(1)
  {
    uint32_t start_tick = osKernelGetTickCount();
    ArmLogic_RC();
    Arm_SetTarget(&arm);
    Arm_Calc(&arm);

    // 发送
    DM_Motor_MIT_Send(&arm.yaw1_8009p);
    DM_Motor_MIT_Send(&arm.pitch1_8009p);
    DM_Motor_MIT_Send(&arm.yaw2_4310);
    DJI_Motor_GroupTransmit(&hcan1, DJI_MOTOR_TX_200);
    osDelayUntil(start_tick + 2);
  }
}

void PushRodTask(void *argument)
{
  {
    PushRod_Init_t push_rod_init = {
        // --- 1. 电机1配置 (M3508 / M2006) ---
        .motor1_init = {
            .hcan = &hcan1,           // 使用 CAN1
            .type = DJI_MOTOR_TYPE_M3508, 
            .tx_id = DJI_MOTOR_TX_200,// 发送 ID 0x200 (控制 ID 1-4)
            .rx_id = 0x201,           // 接收 ID 0x201 (电机 ID 1)
            .dir = DJI_MOTOR_DIR_NORMAL,
            .zero_offset = 0,
            .MotorRxCallback = NULL,
            // .p_owner_moudle 会在 PushRod_Init 内部自动关联
        },
        
        // --- 2. 电机2配置 ---
        .motor2_init = {
            .hcan = &hcan1,
            .type = DJI_MOTOR_TYPE_M3508,
            .tx_id = DJI_MOTOR_TX_200,
            .rx_id = 0x202,           // 接收 ID 0x202 (电机 ID 2)
            .dir = DJI_MOTOR_DIR_REVERSE, // [已修改] 反向安装，令两电机旋转方向相反以协同推出
            .zero_offset = 0,
            .MotorRxCallback = NULL,
        },

        // --- 3. 单电机闭环控制参数 (双环PID) ---
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
            .kp = 0.7f, .ki = 0.f, .kd = 0.0f,
            .out_limit = 16384.0f,
            .i_out_limit = 1000.0f
        },

        // --- 4. 同步控制参数 (Cross-Coupling) ---
        // [A] 位置同步 PID: 修正两电机位置差
        .err_pid_params = {
            .mode = PID_POSITION,
            .kp = 0.f,    // 较大的 P，强行拉回位置
            .ki = 0.0f,   // 消除长期细微偏差
            .kd = 0.0f,   // 抑制同步过程中的震荡
            .out_limit = 0.0f, // 最大同步补偿电流
            .i_out_limit = 1000.0f
        },

        // // [B] 速度同步 PID: 修正两电机速度差 (前馈)
        // .err_spd_pid_params = {
        //     .mode = PID_POSITION,
        //     .kp = 5.0f,    // 速度差更敏感，一旦有速度差立即补偿
        //     .ki = 0.0f,
        //     .kd = 0.0f,
        //     .out_limit = 3000.0f,
        //     .i_out_limit = 0.0f
        // },

        // // [C] 力矩均衡 PID: 平衡两电机电流
        // .err_torque_pid_params = {
        //     .mode = PID_POSITION,
        //     .kp = 0.0f,     // 力矩均衡容易引起震荡，建议先不开 P
        //     .ki = 0.05f,    // 仅靠 I 慢慢拉平两边的负载差
        //     .kd = 0.0f,
        //     .out_limit = 1000.0f, // 限制均衡力度，防止反向出力
        //     .i_out_limit = 1000.0f
        // },

        // --- 5. 其它配置 ---
        .motor1_max_out = 16384.0f,
        .motor2_max_out = 16384.0f,
        .calib_current = -2000,     // 校准时的堵转电流 (负值表示向下/向内收缩)
        .calib_target_count = 1000, // 持续堵转 1s 认为到达零点 (假设 1ms 调用一次)
        // .max_sync_error = 8192.0f * 20.0f // 例如允许最大偏差 20 圈，超过报错
    };
    
    // 执行初始化
    PushRod_Init(&push_rod, &push_rod_init);
    
    // 【修复】初始化后必须使能，否则无法动作
    // 如果需要上电自动找零，改用 PushRod_Calibrate(&push_rod);
    PushRod_Enable(&push_rod);
  }

  // int64_t tar = 0;
  while (1)
  {
      uint32_t start_tick = osKernelGetTickCount();
      // if (tar < 139264 && tar >= -130000) {
      //   PushRod_SetTarget(&push_rod, tar-=100);
      // }

      PushRod_Logic_RC();

      // 2. 核心计算 (1ms 一次)
      PushRod_Calc(&push_rod);

      DJI_Motor_GroupTransmit(&hcan1, DJI_MOTOR_TX_200);

      JustFloat(push_rod.motor2_ctrl.target, *(push_rod.motor2_ctrl.p_pid_ext_fdb), \
                push_rod.motor2_ctrl.pid_external.out, push_rod.motor2.processed_measure.spd_rpm, &huart1);

      osDelayUntil(start_tick + 1); // 保证 1kHz 频率
  }
}

void ChassisTask(void* argument) {
  {
    Chassis_Init_t chassis_init = {
      .motor_rf = {
        .hcan = &hcan2,
        .type = DJI_MOTOR_TYPE_M3508,
        .tx_id = DJI_MOTOR_TX_200,
        .rx_id = 0x202,
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
        .rx_id = 0x201,
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
    Chassis_Logic_RC();
    DJI_Motor_GroupTransmit(&hcan2, DJI_MOTOR_TX_200);
    osDelayUntil(start_tick + 2);
  }
}

void CanTask(void *argument) {
  BSP_CAN_TxInstance can_tx_msg = {0};
  while (1) {
    osMessageQueueGet(canTxMsgQueueHandle, &can_tx_msg, 0, portMAX_DELAY);
    while(BSP_CAN_Transmit(&can_tx_msg) != HAL_OK);
  }
}
