#ifndef CAN_MULTI_COMM_H
#define CAN_MULTI_COMM_H

#include "bsp_can.h"

typedef struct {
  CAN_HandleTypeDef *hcan;
  uint32_t id;
  
} CanMultiComm_Tx_Init_t;

#endif
