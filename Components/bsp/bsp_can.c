#include "bsp_can.h"
#include "main.h"
#include <string.h>
#include "stdlib.h"

/* can instance ptrs storage, used for recv callback */
// 在CAN产生接收中断会遍历数组,选出hcan和rxid与发生中断的实例相同的那个,调用其回调函数
// @todo: 后续为每个CAN总线单独添加一个can_instance指针数组,提高回调查找的性能
static BSP_CAN_RxInstance *g_can_rx_instances[BSP_CAN_MAX_REGISTER_CNT] = {NULL};
static uint8_t g_can_rx_instance_idx; // 全局CAN实例索引,每次有新的模块注册会自增


/// @brief 配置新的can过滤器, 只供本模块使用(目前只实现了StdID)
/// @param rx_instance 接收实例
static void _BSP_CAN_AddFilter(BSP_CAN_RxInstance *rx_instance)
{
  CAN_FilterTypeDef can_filter_conf;
  static uint8_t can1_filter_idx = 0, can2_filter_idx = 14; // 0-13给can1用,14-27给can2用

  can_filter_conf.FilterMode = CAN_FILTERMODE_IDLIST;
  can_filter_conf.FilterScale = CAN_FILTERSCALE_16BIT;
  can_filter_conf.FilterFIFOAssignment = (rx_instance->rx_id & 1) ? CAN_RX_FIFO0 : CAN_RX_FIFO1;              // 奇数id的模块会被分配到FIFO0,偶数id的模块会被分配到FIFO1
  can_filter_conf.SlaveStartFilterBank = 14;                                                                // 从第14个过滤器开始配置从机过滤器(在STM32的BxCAN控制器中CAN2是CAN1的从机)
  can_filter_conf.FilterIdLow = rx_instance->rx_id << 5;                                                      // 过滤器寄存器的低16位,因为使用STDID,所以只有低11位有效,高5位要填0
  can_filter_conf.FilterBank = rx_instance->can_handle == &hcan1 ? (can1_filter_idx++) : (can2_filter_idx++); // 根据can_handle判断是CAN1还是CAN2,然后自增
  can_filter_conf.FilterActivation = CAN_FILTER_ENABLE;                                                     // 启用过滤器

  HAL_CAN_ConfigFilter(rx_instance->can_handle, &can_filter_conf);
}

/// @brief 
void BSP_CAN_InitAll()
{
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);
  HAL_CAN_Start(&hcan2);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
}


void BSP_CAN_RxRegister(BSP_CAN_RxInstance *g_can_rx_instance, CAN_HandleTypeDef *const hcan,
                        const uint32_t rx_id, void *const owner_moudle,
                        void (*can_rx_callback)(struct _RxInstance *))
{ // 超过最大负载，卡死
  if (g_can_rx_instance_idx >= BSP_CAN_MAX_REGISTER_CNT)
  {
    while (1)
    {
    }
  }
  for (size_t i = 0; i < g_can_rx_instance_idx; i++)
  {
    if (g_can_rx_instances[i]->rx_id == rx_id && g_can_rx_instances[i]->can_handle == hcan)
    {
      while (1)
      {
      }
    }
  }

  // 设置回调函数和接收发送id
  g_can_rx_instance->can_handle = hcan;
  // instance->tx_id = config->tx_id; // 好像没用,可以删掉
  g_can_rx_instance->rx_id = rx_id;
  g_can_rx_instance->can_rx_callback = can_rx_callback;
  g_can_rx_instance->owner_moudle = owner_moudle;

  memset(g_can_rx_instance->rx_buff, 0, sizeof(g_can_rx_instance->rx_buff));

  _BSP_CAN_AddFilter(g_can_rx_instance);                           // 添加CAN过滤器规则
  g_can_rx_instances[g_can_rx_instance_idx++] = g_can_rx_instance; // 将实例保存到can_instance中
}

void BSP_CAN_Tx_Init(BSP_CAN_TxInstance *tx_instance, CAN_HandleTypeDef *hcan, uint32_t tx_id, CAN_TxHeaderTypeDef tx_header)
{
  tx_instance->can_handle = hcan; // 设置can句柄
  tx_instance->tx_id = tx_id;     // 设置发送id

  // 进行发送报文的配置
  tx_instance->tx_header.StdId = tx_header.StdId;
  tx_instance->tx_header.ExtId = tx_header.ExtId;
  tx_instance->tx_header.IDE = tx_header.IDE;
  tx_instance->tx_header.RTR = tx_header.RTR;
  tx_instance->tx_header.DLC = tx_header.DLC;

  memset(tx_instance->tx_buff, 0, sizeof(tx_instance->tx_buff));
}

// 目前tx_buff由发送实例保存，但这样做会增加一次复制的性能开销
uint8_t BSP_CAN_Transmit(BSP_CAN_TxInstance *tx_instance)
{
  static uint32_t busy_count;
  if (HAL_CAN_AddTxMessage(tx_instance->can_handle, &tx_instance->tx_header, tx_instance->tx_buff, &tx_instance->tx_mailbox) != HAL_OK)
  {
    // 发送失败就直接返回，不采用阻塞发送
    busy_count++;
    return 0;
  }
  return 1; // 发送成功
}

void BSP_CAN_SetTxDLC(BSP_CAN_TxInstance *tx_instance, uint8_t length)
{
  // 发送长度错误, 置为默认值 : 8
  if (length > 8 || length == 0)
  {
    length = 8;
  }

  tx_instance->tx_header.DLC = length;
}

static void BSP_CAN_Rx_FIFOxCallback(CAN_HandleTypeDef *hcan, uint32_t fifox)
{
  static CAN_RxHeaderTypeDef rx_header;
  uint8_t can_rx_buff[8];
  while (HAL_CAN_GetRxFifoFillLevel(hcan, fifox)) // 阻塞获取所有数据
  {
    HAL_CAN_GetRxMessage(hcan, fifox, &rx_header, can_rx_buff); // 从FIFO中获取数据
    for (size_t i = 0; i < g_can_rx_instance_idx; ++i)
    {
      if (hcan == g_can_rx_instances[i]->can_handle && rx_header.StdId == g_can_rx_instances[i]->rx_id)
      {
        // 若回调不为空就调用
        if (g_can_rx_instances[i]->can_rx_callback != NULL)
        {
          g_can_rx_instances[i]->rx_len = rx_header.DLC;                      // 保存接收到的数据长度
          memcpy(g_can_rx_instances[i]->rx_buff, can_rx_buff, rx_header.DLC); // 消息拷贝到对应实例
          g_can_rx_instances[i]->can_rx_callback(g_can_rx_instances[i]);      // 触发回调进行数据解析和处理
        }
        return;
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
