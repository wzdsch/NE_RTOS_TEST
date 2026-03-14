#ifndef USR_FREERTOS_H
#define USR_FREERTOS_H

#include "cmsis_os.h"

extern osThreadId_t armTaskHandle;
extern osThreadId_t bspCan1TaskHandle;
extern osThreadId_t bspCan2TaskHandle;
extern osThreadId_t canCustomCommTaskHandle;
extern osThreadId_t robotCtrlTaskHandle;

extern osMessageQueueId_t bspCan1TxMsgQueueHandle;
extern osMessageQueueId_t bspCan2TxMsgQueueHandle;

void Usr_FreeRTOS_Init(void);

#endif
