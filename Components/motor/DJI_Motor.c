#include "DJI_Motor.h"

void DJI_Motor_Init(DJI_Motor_t* p_motor, CAN_HandleTypeDef *p_hcan, DJI_Motor_Type_t type, \
                    uint32_t rx_id, void (*callback)(DJI_Motor_t*)) {
  p_motor->p_hcan = p_hcan;
  p_motor->motor_type = type;
  p_motor->rx_id = rx_id;
  p_motor->MotorRxCallback = callback;

  
}
