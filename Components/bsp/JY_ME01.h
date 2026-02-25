#ifndef JY_ME01_H
#define JY_ME01_H

#include "usart.h"

/// @brief 自动回传模式ModBus协议
/// rxd[0]  rxd[1]  rxd[2]  rxd[3] ~ rxd[n-3]  rxd[n-2] rxd[n-1]
///  ID      cmd     len    ------data-------  CRC_Low  CRC_High
typedef struct {
  uint8_t ID;
  uint8_t CMD;
  uint8_t len;
  uint32_t data;
  uint16_t CRC16;
} JY_ME01_ModBus_AutoRxData_t;

typedef struct {
  UART_HandleTypeDef* huart;
  uint8_t rx_buf[9];
  float angle;
  JY_ME01_ModBus_AutoRxData_t auto_rx_data;
  uint32_t err_cnt;
} JY_ME01_ModBus_t;

extern JY_ME01_ModBus_t JY_ME01;

void JY_ME01_Init(JY_ME01_ModBus_t* p_modbus,UART_HandleTypeDef* huart);


#endif
