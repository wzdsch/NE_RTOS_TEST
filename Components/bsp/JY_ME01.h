/*
 * @Author: Jiang Tianhang 1919524828@qq.com
 * @Date: 2026-02-18 09:56:57
 * @LastEditors: Jiang Tianhang 1919524828@qq.com
 * @LastEditTime: 2026-03-05 23:18:31
 * @FilePath: \NE_RTOS_TEST\Components\bsp\JY_ME01.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef JY_ME01_H
#define JY_ME01_H

#include "usart.h"

#define JY_ME01_FRAME_SIZE 9

typedef enum {
  JY_ME01_DIR_NORMAL = 0,
  JY_ME01_DIR_REVERSE = 1,
} JY_ME01_Dir_e;

/// @brief 自动回传模式ModBus协议
/// data[0] data[1] data[2] data[3] ~ data[n-3] data[n-2] data[n-1]
///  ID      cmd     len    ------data-------    CRC_Low   CRC_High
typedef struct {
  uint8_t ID;
  uint8_t CMD;
  uint8_t len;
  uint32_t data;
  uint16_t CRC16;
} JY_ME01_ModBus_AutoRxData_t;

typedef struct {
  UART_HandleTypeDef* huart;
  uint8_t ID;
  float zero_angle;
  JY_ME01_Dir_e dir;
  uint8_t rx_buf[JY_ME01_FRAME_SIZE];
  uint8_t start_idx; // 帧在rx_buf中的起始索引
  float angle;
  float processed_angle;
  JY_ME01_ModBus_AutoRxData_t auto_rx_data;
  uint32_t err_cnt;
} JY_ME01_ModBus_t;

typedef struct {
  UART_HandleTypeDef* huart;
  uint8_t ID;
  float zero_angle;
  JY_ME01_Dir_e dir;
  float init_angle;
} JY_ME01_Init_t;

extern JY_ME01_ModBus_t JY_ME01;

void JY_ME01_Init(JY_ME01_ModBus_t* p_modbus, JY_ME01_Init_t* p_init);

void JY_ME01_RxIDLE_Callback(JY_ME01_ModBus_t* p_modbus);

#endif
