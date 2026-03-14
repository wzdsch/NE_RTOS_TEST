/*
 * @beforeAnnotation: 
 * Copyright (c) 2026 by 
 * """ The Robomaster team : NEXT-E from Xi'an University of Technology """
 * All Rights Reserved. 
 * 
 * @Author: Jiang Tianhang 1919524828@qq.com
 * @Date: 2026-01-05 21:29:25
 * @LastEditors: Jiang Tianhang 1919524828@qq.com
 * @LastEditTime: 2026-03-13 21:28:46
 * @FilePath: \NE_RTOS_TEST\Components\moudle\can_custom_comm.c
 * @Description: 
 */
#include "can_custom_comm.h"
#include <string.h>
#include <stdlib.h>

#include "cmsis_os.h"
extern osMessageQueueId_t canTxMsgQueueHandle;

#define CAN_CUSTOM_COMM_MAX_PACK_CNT 4 // 最大拆包数

// --------------------------------- TX ---------------------------------

void CAN_CustomComm_Tx_Init(CAN_CustomComm_Tx_t* p_tx, CAN_CustomComm_Tx_Init_t* init) {
  uint8_t err = 0;
  if (init == NULL || init->hcan == NULL || init->p_buf == NULL) {
    err++;
  }
  if (init->hcan != &hcan1 && init->hcan != &hcan2) {
    err++;
  }
  if (init->IDE != CAN_ID_STD && init->IDE != CAN_ID_EXT) {
    err++;
  }
  if (init->size == 0) {
    err++;
  }
  if (init->size > CAN_CUSTOM_COMM_MAX_PACK_CNT * 8) {
    err++;
  }

  // 计算拆包数
  p_tx->pack_cnt = (init->size - 1) / 8 + 1;

  if(init->start_tx_id + p_tx->pack_cnt > (init->IDE == CAN_ID_STD ? 0x7FF : 0x1FFFFFFF)) {
    err++; // id超范围
  }

  if (err) {
    while (1) {
      // param err
    }
  }

  BSP_CAN_Tx_Init(&p_tx->tx_instance, init->hcan, init->start_tx_id, init->IDE, 8, CAN_RTR_DATA);
  p_tx->start_tx_id = init->start_tx_id;
  p_tx->p_buf = init->p_buf;
  p_tx->size = init->size;
  p_tx->pPackFunc = init->pPackFunc;
}

void CAN_CustomComm_Tx_PackSend(CAN_CustomComm_Tx_t* p_tx) {
  if (p_tx->pPackFunc != NULL) {
    p_tx->pPackFunc(p_tx->p_buf); // 打包
  }
  BSP_CAN_SetTxID(&p_tx->tx_instance, p_tx->start_tx_id); // 恢复初始id
  
  uint8_t send_size = 0; // 暂时发送的字节数
  // 拆包并发送
  while (send_size < p_tx->size) {
    // 当前数据包的大小 (1 ~ 8)
    uint8_t pack_size = (p_tx->size - send_size) >= 8 ? 8 : (p_tx->size - send_size);
    BSP_CAN_SetTxDLC(&p_tx->tx_instance, pack_size);
    BSP_CAN_SetTxBuf(&p_tx->tx_instance, (uint8_t*)p_tx->p_buf + send_size);

    BSP_CAN_Transmit(&p_tx->tx_instance);

    BSP_CAN_SetTxID(&p_tx->tx_instance, p_tx->tx_instance.tx_id + 1); // 每发一个包id自增1
    send_size += pack_size;
  }
}

// --------------------------------- RX ---------------------------------

static void _CAN_CustomComm_RxCallback(BSP_CAN_RxInstance* p_rx_instance);

void CAN_CustomComm_Rx_Init(CAN_CustomComm_Rx_t* p_rx, CAN_CustomComm_Rx_Init_t* init) {
  uint8_t err = 0;
  if (init == NULL || init->hcan == NULL || init->p_buf == NULL) {
    err++;
  }
  if (init->hcan != &hcan1 && init->hcan != &hcan2) {
    err++;
  }
  if (init->IDE != CAN_ID_STD && init->IDE != CAN_ID_EXT) {
    err++;
  }
  if (init->size == 0) {
    err++;
  }
  if (init->size > CAN_CUSTOM_COMM_MAX_PACK_CNT * 8) {
    err++; // 数据包太大, 拆包太多
  }

  // 计算拆包数
  p_rx->pack_cnt = (init->size - 1) / 8 + 1;

  // 申请接收实例数组内存
  p_rx->p_rx_instances = (BSP_CAN_RxInstance*)calloc(p_rx->pack_cnt, sizeof(BSP_CAN_RxInstance));
  if (p_rx->p_rx_instances == NULL) {
    err++; // 内存分配失败
  }

  if (init->start_rx_id + p_rx->pack_cnt > (init->IDE == CAN_ID_STD ? 0x7FF : 0x1FFFFFFF)) {
    err++; // id超范围
  }
  if (err) {
    while (1) {
      // param err
    }
  }

  p_rx->start_rx_id = init->start_rx_id;
  p_rx->last_rx_id = 0x1FFFFFFF; // 初始化为一个不可能的id, 以便接收第一个包时进行正确的连续性检查
  p_rx->err_cnt = 0;
  p_rx->p_buf = init->p_buf;
  p_rx->size = init->size;
  p_rx->pUnpackFunc = init->pUnpackFunc;

  // 注册接收实例
  for (int i = 0; i < p_rx->pack_cnt; i++) {
    BSP_CAN_RxRegister(&p_rx->p_rx_instances[i], init->hcan, init->start_rx_id + i, init->IDE, p_rx, _CAN_CustomComm_RxCallback);
  }
}

static void _CAN_CustomComm_RxCallback(BSP_CAN_RxInstance* p_rx_instance) {
  CAN_CustomComm_Rx_t*p_custom_rx = (CAN_CustomComm_Rx_t*)(p_rx_instance->p_owner_moudle);

  // 错误时丢弃读取到的数据, 等下一组新数据
  // 两种可能丢包/出错的情况:
  // 1. 接收到的包不是起始包时, 与上次接收的包id不连续
  // 2. 接收到的包id超过范围(一般不会出现, 但作为保险措施)
  if ((p_rx_instance->rx_id != p_custom_rx->start_rx_id && \
      p_rx_instance->rx_id != p_custom_rx->last_rx_id + 1) || \
      p_rx_instance->rx_id >= p_custom_rx->start_rx_id + p_custom_rx->pack_cnt) {
    p_custom_rx->err_cnt++;
    return;
  }

  // 按照ID顺序拼接数据
  memcpy((uint8_t*)p_custom_rx->p_buf + (p_rx_instance->rx_id - p_custom_rx->start_rx_id) * 8, \
         p_rx_instance->rx_buff, p_rx_instance->rx_len);
  p_custom_rx->last_rx_id = p_rx_instance->rx_id; // 更新上次接收的ID
  
  // 若是最后一个包，解包
  if ((p_rx_instance->rx_id - p_custom_rx->start_rx_id + 1) == p_custom_rx->pack_cnt) {
    if (p_custom_rx->pUnpackFunc != NULL) {
      p_custom_rx->pUnpackFunc(p_custom_rx->p_buf);
    }
  }
}
