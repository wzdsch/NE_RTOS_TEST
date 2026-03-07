/*
 * @beforeAnnotation: 
 * Copyright (c) 2026 by 
 * """ The Robomaster team : NEXT-E from Xi'an University of Technology """
 * All Rights Reserved. 
 * 
 * @Author: Jiang Tianhang 1919524828@qq.com
 * @Date: 2025-11-16 18:39:25
 * @LastEditors: Jiang Tianhang 1919524828@qq.com
 * @LastEditTime: 2026-03-04 17:20:54
 * @FilePath: \MDK-ARMd:\RoboMaster\code\NE_RTOS_TEST\Components\bsp\bsp_can.c
 * @Description: 
 */
#include "bsp_can.h"
#include "main.h"
#include <string.h>
#include "stdlib.h"

#include "uthash.h" // 这是一个快速查找键值对的库, 后续考虑要不要用来代替接收时遍历，我在使用HASH_FIND_INT时出现了bug

/* can instance ptrs storage, used for recv callback */
// 在CAN产生接收中断会遍历数组,选出hcan和rxid与发生中断的实例相同的那个,调用其回调函数
// @todo: 后续为每个CAN总线单独添加一个can_instance指针数组,提高回调查找的性能
static BSP_CAN_RxInstance *gp_can_rx_instances[BSP_CAN_MAX_REGISTER_CNT] = {NULL};
static uint8_t g_can_rx_instance_idx; // 全局CAN实例索引,每次有新的模块注册会自增


/// @brief 配置新的can过滤器, 只供本模块使用
/// @param rx_instance 接收实例
static void _BSP_CAN_AddFilter(BSP_CAN_RxInstance *rx_instance)
{
  CAN_FilterTypeDef can_filter_conf;
  static uint8_t can1_filter_idx = 0, can2_filter_idx = 14; // 0-13给can1用,14-27给can2用

  can_filter_conf.FilterMode = CAN_FILTERMODE_IDLIST; // 列表模式

  // 标准帧
  if (rx_instance->IDE == CAN_ID_STD) {
    can_filter_conf.FilterScale = CAN_FILTERSCALE_16BIT;

    // 过滤器寄存器的低16位,因为使用STDID,所以只有高11位有效,低5位要填0
    can_filter_conf.FilterIdLow = rx_instance->rx_id << 5 | CAN_ID_STD;
  }

  // 扩展帧
  else {
    can_filter_conf.FilterScale = CAN_FILTERSCALE_32BIT;

    // 过滤器寄存器的低32位
    can_filter_conf.FilterIdHigh = (rx_instance->rx_id >> 13) & 0xFFFF;
    can_filter_conf.FilterIdLow = (rx_instance->rx_id << 3) & 0xFFFF | CAN_ID_EXT;
  }

  // 奇数id的模块会被分配到FIFO0,偶数id的模块会被分配到FIFO1
  can_filter_conf.FilterFIFOAssignment = (rx_instance->rx_id & 1) ? CAN_FILTER_FIFO0 : CAN_FILTER_FIFO1;

  // 从第14个过滤器开始配置从机过滤器(在STM32的BxCAN控制器中CAN2是CAN1的从机)
  can_filter_conf.SlaveStartFilterBank = 14;

  // 根据can_handle判断是CAN1还是CAN2,然后自增
  can_filter_conf.FilterBank = rx_instance->p_can_handle == &hcan1 ? (can1_filter_idx++) : (can2_filter_idx++);
  
  can_filter_conf.FilterActivation = CAN_FILTER_ENABLE; // 启用过滤器

  HAL_CAN_ConfigFilter(rx_instance->p_can_handle, &can_filter_conf);
}

/// @brief 
void BSP_CAN_InitAll(void)
{
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);
  HAL_CAN_Start(&hcan2);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
}

void BSP_CAN_RxRegister(BSP_CAN_RxInstance *gp_can_rx_instance, CAN_HandleTypeDef *const hcan,
                        const uint32_t rx_id, const uint32_t IDE, void *const owner_moudle,
                        void (*pCanRxCallback)(BSP_CAN_RxInstance *))
{
  // 超过最大负载，卡死
  if (g_can_rx_instance_idx >= BSP_CAN_MAX_REGISTER_CNT)
  {
    while (1)
    {
    }
  }
  for (size_t i = 0; i < g_can_rx_instance_idx; i++)
  {
    if (gp_can_rx_instances[i]->rx_id == rx_id && gp_can_rx_instances[i]->p_can_handle == hcan)
    {
      while (1)
      {
      }
    }
  }

  // 设置回调函数和接收发送id
  gp_can_rx_instance->p_can_handle = hcan;
  gp_can_rx_instance->rx_id = rx_id;
  gp_can_rx_instance->IDE = IDE;
  gp_can_rx_instance->pCanRxCallback = pCanRxCallback;
  gp_can_rx_instance->p_owner_moudle = owner_moudle;

  memset(gp_can_rx_instance->rx_buff, 0, sizeof(gp_can_rx_instance->rx_buff)); // 清空接收缓存

  _BSP_CAN_AddFilter(gp_can_rx_instance); // 添加CAN过滤器规则
  gp_can_rx_instances[g_can_rx_instance_idx++] = gp_can_rx_instance; // 将实例保存到can_instance中
}

void BSP_CAN_Tx_Init(BSP_CAN_TxInstance *p_tx_instance, CAN_HandleTypeDef *p_hcan, uint32_t tx_id, \
                     uint32_t IDE, uint32_t DLC, uint32_t RTR)
{
  // 参数校验
  if (p_tx_instance == NULL || p_hcan == NULL) {
    while (1) {
      // 参数错误
    }
  }

  if (IDE != CAN_ID_STD && IDE != CAN_ID_EXT) {
    while (1) {
      // 参数错误
    }
  }

  if (DLC > 8) {
    while (1) {
      // 参数错误
    }
  }

  if (RTR != CAN_RTR_DATA && RTR != CAN_RTR_REMOTE) {
    while (1) {
      // 参数错误
    }
  }

  p_tx_instance->p_can_handle = p_hcan; // 设置can句柄
  p_tx_instance->tx_id = tx_id;     // 设置发送id
  memset(p_tx_instance->tx_buf, 0, 8); // 清空发送缓存

  // 标准帧id共11位
  if (IDE == CAN_ID_STD) {
    p_tx_instance->tx_header.StdId = tx_id & 0x000007FF; // 11 bits
  }

  // 扩展帧id共29位
  else {
    p_tx_instance->tx_header.ExtId = tx_id & 0x1FFFFFFF; // 29 bits
  }

  p_tx_instance->tx_header.IDE = IDE;
  p_tx_instance->tx_header.DLC = DLC;
  p_tx_instance->tx_header.RTR = RTR;
}

HAL_StatusTypeDef BSP_CAN_Transmit(BSP_CAN_TxInstance *const tx_instance)
{
  static uint32_t busy_count;
  HAL_StatusTypeDef status = HAL_OK;
  if ((status = HAL_CAN_AddTxMessage(tx_instance->p_can_handle, &tx_instance->tx_header, tx_instance->tx_buf, &tx_instance->tx_mailbox)) != HAL_OK)
  {
    // 发送失败就直接返回，不采用阻塞发送
    busy_count++;
    return status;
  }
  return status; // 发送成功
}

inline void BSP_CAN_SetTxDLC(BSP_CAN_TxInstance *p_tx_instance, uint8_t length)
{
  // 发送长度错误, 置为默认值 : 8
  if (length > 8)
  {
    length = 8;
  }

  p_tx_instance->tx_header.DLC = length;
}

inline void BSP_CAN_SetTxID(BSP_CAN_TxInstance *p_tx_instance, uint32_t tx_id) {
  p_tx_instance->tx_id = tx_id;
  if (p_tx_instance->tx_header.IDE == CAN_ID_STD) {
    p_tx_instance->tx_header.StdId = tx_id & 0x000007FF; // 11 bits
  } else if (p_tx_instance->tx_header.IDE == CAN_ID_EXT) {
    p_tx_instance->tx_header.ExtId = tx_id & 0x1FFFFFFF;          // 29 bits
  }
}

inline void BSP_CAN_SetTxBuf(BSP_CAN_TxInstance *p_tx_instance, uint8_t *const p_buf) {
  memcpy(p_tx_instance->tx_buf, p_buf, p_tx_instance->tx_header.DLC);
}

static void BSP_CAN_Rx_FIFOxCallback(CAN_HandleTypeDef *hcan, uint32_t fifox)
{
  CAN_RxHeaderTypeDef rx_header;
  uint8_t can_rx_buff[8];
  while (HAL_CAN_GetRxFifoFillLevel(hcan, fifox)) // 阻塞获取所有数据
  {
    HAL_CAN_GetRxMessage(hcan, fifox, &rx_header, can_rx_buff); // 从FIFO中获取数据
    for (size_t i = 0; i < g_can_rx_instance_idx; ++i)
    {
      // 寻找can IDE ID都匹配的接收实例
      if (hcan == gp_can_rx_instances[i]->p_can_handle && gp_can_rx_instances[i]->IDE == rx_header.IDE && \
        (rx_header.IDE == CAN_ID_STD ? (rx_header.StdId == gp_can_rx_instances[i]->rx_id) : rx_header.ExtId == gp_can_rx_instances[i]->rx_id))
      {
        gp_can_rx_instances[i]->rx_header = rx_header;                       // 保存报头信息

        gp_can_rx_instances[i]->rx_len = rx_header.DLC;                      // 保存接收到的数据长度
        memcpy(gp_can_rx_instances[i]->rx_buff, can_rx_buff, rx_header.DLC); // 消息拷贝到对应实例
        
        // 若回调不为空就调用
        if (gp_can_rx_instances[i]->pCanRxCallback != NULL)
        {
          gp_can_rx_instances[i]->pCanRxCallback(gp_can_rx_instances[i]);      // 触发回调进行数据解析和处理
        }
        break;
      }
    }
  }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  BSP_CAN_Rx_FIFOxCallback(hcan, CAN_RX_FIFO0); // 调用我们自己写的函数来处理消息
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  BSP_CAN_Rx_FIFOxCallback(hcan, CAN_RX_FIFO1); // 调用我们自己写的函数来处理消息
}
