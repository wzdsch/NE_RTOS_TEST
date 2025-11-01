#ifndef DJI_MOTOR_H
#define DJI_MOTOR_H

#include "motor_common.h"
#include "bsp_can.h"

typedef enum
{
  DJI_MOTOR_3508,
  DJI_MOTOR_2006,
  DJI_MOTOR_6020
} DJI_Motor_Type_t;

typedef struct _DJI_Motor_t {
  CAN_HandleTypeDef *p_hcan; // CAN handle
  uint32_t rx_id;
  BSP_CAN_RxInstance can_rx_instance;
  DJI_Motor_Type_t motor_type;

  MotorCommon_t common_data;
  void (*MotorRxCallback)(struct _DJI_Motor_t *);
} DJI_Motor_t;

#endif
