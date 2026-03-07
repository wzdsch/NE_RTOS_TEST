/*
 * @beforeAnnotation:
 * Copyright (c) 2026 by
 * """ The Robomaster team : NEXT-E from Xi'an University of Technology """
 * All Rights Reserved.
 *
 * @Author: Jiang Tianhang 1919524828@qq.com
 * @Date: 2026-01-07 11:36:00
 * @LastEditors: Jiang Tianhang 1919524828@qq.com
 * @LastEditTime: 2026-03-07 14:40:52
 * @FilePath: \NE_RTOS_TEST\Components\remote_ctrl\remote_receive.c
 * @Description: This file is by Guo Hongting
 */
#include "remote_receive.h"
#include <stdbool.h>
#include "usart.h"

extern UART_HandleTypeDef huart3;        // uart3句柄
extern DMA_HandleTypeDef hdma_usart3_rx; // dma对应uart句柄

// 最终拆包后的数据
FSI6Data_t fsi6_data = {
  .left_x = FSI6_CHANNEL_MID,
  .left_y = FSI6_CHANNEL_MID,
  .right_x = FSI6_CHANNEL_MID,
  .right_y = FSI6_CHANNEL_MID,
  .left_ch1 = FSI6_CHANNEL_MIN,
  .left_ch2 = FSI6_CHANNEL_MIN,
  .right_ch1 = FSI6_CHANNEL_MIN,
  .right_ch2 = FSI6_CHANNEL_MIN,
  .left_knob = FSI6_CHANNEL_MIN,
  .right_knob = FSI6_CHANNEL_MIN,
};
__attribute__((section(".sram2"))) uint8_t W_BusRxBuffer[2][RC_FRAME_NUM]; // DMA 双缓冲区
__attribute__((section(".sram2"))) uint8_t *fsi6_completed_buf;            // 指向本次接收完成的数据缓冲区

void GetFSI6Data(FSI6Data_t *fsi6Data, uint8_t *rxBuffer)
{
  fsi6Data->fsi6_start = rxBuffer[0];
  fsi6Data->right_x = ((uint16_t)rxBuffer[1] >> 0 | ((uint16_t)rxBuffer[2] << 8)) & 0x07FF;
  fsi6Data->right_y = ((uint16_t)rxBuffer[2] >> 3 | ((uint16_t)rxBuffer[3] << 5)) & 0x07FF;
  fsi6Data->left_y = ((uint16_t)rxBuffer[3] >> 6 | ((uint16_t)rxBuffer[4] << 2) | (uint16_t)rxBuffer[5] << 10) & 0x07FF;
  fsi6Data->left_x = ((uint16_t)rxBuffer[5] >> 1 | ((uint16_t)rxBuffer[6] << 7)) & 0x07FF;
  fsi6Data->left_ch1 = ((uint16_t)rxBuffer[6] >> 4 | ((uint16_t)rxBuffer[7] << 4)) & 0x07FF;
  fsi6Data->left_ch2 = ((uint16_t)rxBuffer[7] >> 7 | ((uint16_t)rxBuffer[8] << 1) | (uint16_t)rxBuffer[9] << 9) & 0x07FF;
  fsi6Data->right_ch2 = ((uint16_t)rxBuffer[9] >> 2 | ((uint16_t)rxBuffer[10] << 6)) & 0x07FF;
  fsi6Data->right_ch1 = ((uint16_t)rxBuffer[10] >> 5 | ((uint16_t)rxBuffer[11] << 3)) & 0x07FF;
  fsi6Data->left_knob = ((uint16_t)rxBuffer[12] >> 0 | ((uint16_t)rxBuffer[13] << 8)) & 0x07FF;
  fsi6Data->right_knob = ((uint16_t)rxBuffer[13] >> 3 | ((uint16_t)rxBuffer[14] << 5)) & 0x07FF;
  fsi6Data->fsi6_flag = rxBuffer[23];
  fsi6Data->fsi6_end = rxBuffer[24];
}

/* 初始化串口3的DMA接收功能 并配置双缓冲区模式 */
// 参数
// rx0_buf：内存缓冲区0的地址，用于存储接收到的数据
// rx1_buf：内存缓冲区1的地址，用于存储接收到的数据
// dma_buf_num：DMA要传输的数据长度
void FSI6_BUS_IDLEHandler_Init()
{
  // 使能DMA串口接收功能
  // 通过设置USART3的控制寄存器3（CR3）的DMAR位，允许DMA直接从串口接收数据
  SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR);
  // 使能串口3的空闲中断
  // 当串口接收数据出现空闲时会触发该中断
  __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
  // 关闭DMA传输
  // 确保后续配置不会受到之前DMA操作的影响
  __HAL_DMA_DISABLE(&hdma_usart3_rx);
  // 设置DMA的外设地址
  // 将DMA的外设地址设置为USART3的数据寄存器地址，DMA从该寄存器读取数据
  hdma_usart3_rx.Instance->PAR = (uint32_t)&(USART3->DR);
  // 设置DMA的内存缓冲区0的地址
  // DMA将接收到的数据存储到该缓冲区
  hdma_usart3_rx.Instance->M0AR = (uint32_t)(W_BusRxBuffer[0]);
  // 设置DMA的内存缓冲区1的地址
  // DMA在两个缓冲区之间切换存储接收到的数据
  hdma_usart3_rx.Instance->M1AR = (uint32_t)(W_BusRxBuffer[1]);
  // 设置DMA要传输的数据长度
  // 指定本次DMA传输的数据数量
  hdma_usart3_rx.Instance->NDTR = RC_FRAME_NUM;
  // 使能DMA的双缓冲区模式
  // DMA会在两个缓冲区之间自动切换，提高数据处理效率
  SET_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_DBM);
  // 开启DMA传输
  // 开始从串口接收数据到指定的缓冲区
  __HAL_DMA_ENABLE(&hdma_usart3_rx);
}

// 串口空闲中断接收
void FSI6_BUS_IDLEHandler(void)
{
  if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE))
  {
    __HAL_UART_CLEAR_IDLEFLAG(&huart3); // 必须清除，否则下次不会触发
    // 暂停 DMA，防止数据还在写入
    __HAL_DMA_DISABLE(&hdma_usart3_rx);
    // 判断当前正在使用哪个缓冲区
    if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
    {
      fsi6_completed_buf = W_BusRxBuffer[0];
    }
    else
    {
      fsi6_completed_buf = W_BusRxBuffer[1];
    }
    GetFSI6Data(&fsi6_data, fsi6_completed_buf);
    // 重新启动 DMA 接收
    __HAL_DMA_SET_COUNTER(&hdma_usart3_rx, RC_FRAME_NUM);
    __HAL_DMA_ENABLE(&hdma_usart3_rx);
  }
}
