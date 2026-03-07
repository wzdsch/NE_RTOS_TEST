/*
 * @Author: Jiang Tianhang 1919524828@qq.com
 * @Date: 2026-02-18 09:56:41
 * @LastEditors: Jiang Tianhang 1919524828@qq.com
 * @LastEditTime: 2026-03-06 16:07:25
 * @FilePath: \NE_RTOS_TEST\Components\bsp\JY_ME01.c
 * @Description: JY-ME01编码器ModBus自动反馈模式, DMA必须配置为循环模式
 */
#include "JY_ME01.h"
#include "string.h"

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

void JY_ME01_Init(JY_ME01_ModBus_t* p_modbus, JY_ME01_Init_t* p_init) {
  if (p_modbus == NULL || p_init == NULL) {
    while (1) {
      // error
    }
  }
  p_modbus->huart = p_init->huart;
  p_modbus->ID = p_init->ID;
  p_modbus->dir = p_init->dir;
  p_modbus->zero_angle = p_init->zero_angle;
  p_modbus->angle = p_init->init_angle;
  p_modbus->processed_angle = p_init->init_angle;
  memset(&(p_modbus->auto_rx_data), 0, sizeof(JY_ME01_ModBus_AutoRxData_t));
  memset(p_modbus->rx_buf, 0, sizeof(p_modbus->rx_buf));
  p_modbus->start_idx = 0;
  p_modbus->err_cnt = 0;
  __HAL_UART_ENABLE_IT(p_modbus->huart, UART_IT_IDLE);
  HAL_UARTEx_ReceiveToIdle_DMA(p_modbus->huart, p_modbus->rx_buf, JY_ME01_FRAME_SIZE);
}

void JY_ME01_RxIDLE_Callback(JY_ME01_ModBus_t* p_modbus) {
  if (__HAL_UART_GET_FLAG(p_modbus->huart, UART_FLAG_IDLE) != RESET) {
    // 清除 IDLE 标志位
    __HAL_UART_CLEAR_IDLEFLAG(p_modbus->huart);
  }

  // 停止 DMA
  // HAL_UART_DMAPause(p_modbus->huart);

  uint8_t found = 0; // 是否成功接收

  // 找包头 + CRC + 解包
  for (uint8_t find_cnt = 0; find_cnt < JY_ME01_FRAME_SIZE; find_cnt++) {
    if (p_modbus->rx_buf[p_modbus->start_idx] == p_modbus->ID) {
      p_modbus->auto_rx_data.CRC16 = p_modbus->rx_buf[(p_modbus->start_idx + 7) % JY_ME01_FRAME_SIZE] | \
                                     p_modbus->rx_buf[(p_modbus->start_idx + 8) % JY_ME01_FRAME_SIZE] << 8;
      uint8_t temp_buf[7] = {
        p_modbus->rx_buf[p_modbus->start_idx],
        p_modbus->rx_buf[(p_modbus->start_idx + 1) % JY_ME01_FRAME_SIZE],
        p_modbus->rx_buf[(p_modbus->start_idx + 2) % JY_ME01_FRAME_SIZE],
        p_modbus->rx_buf[(p_modbus->start_idx + 3) % JY_ME01_FRAME_SIZE],
        p_modbus->rx_buf[(p_modbus->start_idx + 4) % JY_ME01_FRAME_SIZE],
        p_modbus->rx_buf[(p_modbus->start_idx + 5) % JY_ME01_FRAME_SIZE],
        p_modbus->rx_buf[(p_modbus->start_idx + 6) % JY_ME01_FRAME_SIZE]
      };
      if (ModBus_CRC16(temp_buf, 7) == p_modbus->auto_rx_data.CRC16) {
        // 校验成功 解包
        p_modbus->auto_rx_data.ID = temp_buf[0];
        p_modbus->auto_rx_data.CMD = temp_buf[1];
        p_modbus->auto_rx_data.len = temp_buf[2];

        // 数据拼接
        p_modbus->auto_rx_data.data = temp_buf[3] << 24 | temp_buf[4] << 16 | \
                                      temp_buf[5] << 8 | temp_buf[6];
        // 按照官方的文档解包
        p_modbus->angle = p_modbus->auto_rx_data.data / 262144.f * 360.f;

        // 处理方向和零点
        float tmp_angle = p_modbus->angle - p_modbus->zero_angle;

        // 角度限制在0~360
        if (tmp_angle >= 360.f) {
          tmp_angle -= 360.f;
        }
        if (tmp_angle < 0.f) {
          tmp_angle += 360.f;
        }
        if (p_modbus->dir == JY_ME01_DIR_NORMAL) {
          p_modbus->processed_angle = tmp_angle;
        } else {
          p_modbus->processed_angle = 360.f - tmp_angle;
        }
        p_modbus->start_idx += JY_ME01_FRAME_SIZE;
        p_modbus->start_idx %= JY_ME01_FRAME_SIZE;
        found = 1;
        break;
      }
    }

    p_modbus->start_idx++;
    p_modbus->start_idx %= JY_ME01_FRAME_SIZE;
  }

  if (!found) {
    p_modbus->err_cnt++;
  }

  // 开启DMA
  // HAL_UART_DMAResume(p_modbus->huart);
}
