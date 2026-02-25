#include "usr_freertos.h"

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "usr_tasks.h"
#include "bsp_can.h"

osThreadId_t armTaskHandle;
osThreadId_t canTaskHandle;
osMessageQueueId_t canTxMsgQueueHandle;

void Usr_FreeRTOS_Init(void) {
  canTxMsgQueueHandle = osMessageQueueNew(20, sizeof(BSP_CAN_TxInstance), NULL);
  armTaskHandle = osThreadNew(ArmTask, NULL, NULL);
  canTaskHandle = osThreadNew(CanTask, NULL, NULL);
}
