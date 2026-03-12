/*
 * @Author: Jiang Tianhang 1919524828@qq.com
 * @Date: 2026-02-24 11:50:47
 * @LastEditors: Jiang Tianhang 1919524828@qq.com
 * @LastEditTime: 2026-03-12 01:13:40
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

osThreadId_t canTaskHandle;
osThreadAttr_t canTask_attributes = {
  .name = "canTask",
  .stack_size = 512,
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

osMessageQueueId_t canTxMsgQueueHandle;

void Usr_FreeRTOS_Init(void) {
  canTxMsgQueueHandle = osMessageQueueNew(20, sizeof(BSP_CAN_TxInstance), NULL);
  canTaskHandle = osThreadNew(CanTask, NULL, &canTask_attributes);
  chassisTaskHandle = osThreadNew(ChassisTask, NULL, &chassisTask_attributes);
  canCustomCommHandle = osThreadNew(CanCustomCommTask, NULL, &canCustomComm_attributes);
  pushRodTaskHandle = osThreadNew(PushRodTask, NULL, &pushRodTask_attributes);
}
