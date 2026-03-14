/*
 * @Author: Jiang Tianhang 1919524828@qq.com
 * @Date: 2026-02-24 11:50:47
 * @LastEditors: Jiang Tianhang 1919524828@qq.com
 * @LastEditTime: 2026-03-13 22:07:38
 * @FilePath: \NE_RTOS_TEST\Components\app\usr_freertos.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "usr_freertos.h"

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "usr_tasks.h"
#include "bsp_can.h"

osThreadId_t bspCan1TaskHandle;
osThreadAttr_t bspCan1Task_attributes = {
  .name = "bspCan1Task",
  .stack_size = 1024,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t bspCan2TaskHandle;
osThreadAttr_t bspCan2Task_attributes = {
  .name = "bspCan2Task",
  .stack_size = 1024,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t pushRodTaskHandle;
osThreadAttr_t pushRodTask_attributes = {
  .name = "pushRodTask",
  .stack_size = 1024,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t chassisTaskHandle;
osThreadAttr_t chassisTask_attributes = {
  .name = "chassisTask",
  .stack_size = 1024,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t canCustomCommHandle;
osThreadAttr_t canCustomComm_attributes = {
  .name = "canCustomCommTask",
  .stack_size = 512,
  .priority = (osPriority_t) osPriorityNormal,
};

osMessageQueueId_t bspCan1TxMsgQueueHandle;
osMessageQueueId_t bspCan2TxMsgQueueHandle;

void Usr_FreeRTOS_Init(void) {
  bspCan1TxMsgQueueHandle = osMessageQueueNew(20, sizeof(BSP_CAN_TxInstance), NULL);
  bspCan2TxMsgQueueHandle = osMessageQueueNew(20, sizeof(BSP_CAN_TxInstance), NULL);
  bspCan1TaskHandle = osThreadNew(BspCan1Task, NULL, &bspCan1Task_attributes);
  bspCan2TaskHandle = osThreadNew(BspCan2Task, NULL, &bspCan2Task_attributes);
  chassisTaskHandle = osThreadNew(ChassisTask, NULL, &chassisTask_attributes);
  canCustomCommHandle = osThreadNew(CanCustomCommTask, NULL, &canCustomComm_attributes);
  pushRodTaskHandle = osThreadNew(PushRodTask, NULL, &pushRodTask_attributes);
}
