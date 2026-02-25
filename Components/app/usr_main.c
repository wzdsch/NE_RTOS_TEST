#include "usr_main.h"

#include "bsp_can.h"
#include "DJI_Motor.h"
#include "remote_receive.h"

void Usr_Main_Init(void) {
  BSP_CAN_InitAll();
	DJI_Motor_TxInitAll();
  FSI6_BUS_IDLEHandler_Init();
}

