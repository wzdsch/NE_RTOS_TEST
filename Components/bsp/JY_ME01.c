#include "JY_ME01.h"
#include "string.h"

#define JY_ME01_INIT_ANGLE 0.f

JY_ME01_ModBus_t JY_ME01;

uint16_t ModBus_CRC16(uint8_t* data, uint8_t len) {
  uint16_t crc_temp = 0xFFFF;
  for (uint8_t i = 0; i < len; i++) {
    crc_temp ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc_temp & 0x0001) {
        crc_temp >>= 1;
        crc_temp ^= 0xA001; 
      } else {
        crc_temp >>= 1;
      }
    }
  }
  return crc_temp;
}

void JY_ME01_Init(JY_ME01_ModBus_t* p_modbus,UART_HandleTypeDef* huart) {
  if (p_modbus == NULL) {
    while (1) {
      // error
    }
  }
  p_modbus->huart = huart;
  p_modbus->angle = JY_ME01_INIT_ANGLE;
  memset(&(p_modbus->auto_rx_data), 0, sizeof(JY_ME01_ModBus_AutoRxData_t));
  memset(p_modbus->rx_buf, 0, sizeof(p_modbus->rx_buf));
  p_modbus->err_cnt = 0;
  HAL_UART_Receive_DMA(p_modbus->huart, p_modbus->rx_buf, 9);
}

void JY_ME01_RxCallback(JY_ME01_ModBus_t* p_modbus) {
  p_modbus->auto_rx_data.CRC16 = p_modbus->rx_buf[7] | p_modbus->rx_buf[8] << 8;
  if (ModBus_CRC16(p_modbus->rx_buf, 7) == p_modbus->auto_rx_data.CRC16) {
    p_modbus->auto_rx_data.ID = p_modbus->rx_buf[0];
    p_modbus->auto_rx_data.CMD = p_modbus->rx_buf[1];
    p_modbus->auto_rx_data.len = p_modbus->rx_buf[2];

    // 数据拼接
    p_modbus->auto_rx_data.data = p_modbus->rx_buf[3] << 24 | p_modbus->rx_buf[4] << 16 | \
                                  p_modbus->rx_buf[5] << 8 | p_modbus->rx_buf[6];
    // 按照官方的文档解包
    p_modbus->angle = p_modbus->auto_rx_data.data / 262144.f * 360.f;
  } else {
    p_modbus->err_cnt++;
  }
  HAL_UART_Receive_DMA(p_modbus->huart, p_modbus->rx_buf, 9);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart == JY_ME01.huart) {
    JY_ME01_RxCallback(&JY_ME01);
  }
}
