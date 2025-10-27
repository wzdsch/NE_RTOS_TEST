#ifndef DOUBLE_BUFFER_H
#define DOUBLE_BUFFER_H

#include <stdint.h>

/*  状态转换图

  -------------------<4---------------------
  |                                        |
IDLE --1>-- WRITING --2>-- WRITED --3>-- READING
              |              |
              ------<5--------
1. 若在有空闲状态的缓冲区时收到数据，则写入空闲缓冲区，并进入WRITING状态
2. 写入完成，进入WRITED状态，并将last_writed置为当前缓冲区
3. WRITED状态的缓冲区可被读取，读取请求发送，则读取last_writed缓冲区，进入READING状态
4. 读取完成，进入IDLE状态
5. 若两个缓冲区都不处于IDLE状态时收到数据，则接收数据到非last_writed缓冲区，并进入WRITING状态

  状态转换说明：
- IDLE → WRITING：开始写入
- WRITING → WRITED：写入完成
- WRITED → READING：开始读取
- READING → IDLE：读取完成
- WRITED → WRITING：覆盖写入（双缓冲均满时）
*/

// 缓冲区状态
typedef enum {
    _BUFFER_STATE_IDLE,     // 空闲（可写入）
    _BUFFER_STATE_READING,  // 读取中（禁止写入）
    _BUFFER_STATE_WRITING,  // 写入中（禁止读取）
    _BUFFER_STATE_WRITED    // 写入完成（可读取）
} _Buffer_State_e;

// 缓冲区内存分配状态
typedef enum {
    DOUBLE_BUFFER_MALLOC_NONE,    // 未分配内存
    DOUBLE_BUFFER_MALLOC_DONE     // 已分配内存
} DoubleBuffer_MallocState_e;

// 操作结果
typedef enum {
    DOUBLE_BUFFER_RETURN_SUCCESS,    // 操作成功
    DOUBLE_BUFFER_RETURN_NULLPTR,    // 空指针错误
    DOUBLE_BUFFER_RETURN_NOMEM,      // 未分配内存
    DOUBLE_BUFFER_RETURN_NOTARGET    // 无可用缓冲区（读写目标不存在）
} DoubleBuffer_Return_e;

// 内部缓冲区结构体（仅模块内部使用）
typedef struct {
    _Buffer_State_e state;  // 缓冲区状态
    void *buf;         // 数据存储区
} _Buffer_t;

// 双缓冲区核心结构体
typedef struct {
    DoubleBuffer_MallocState_e malloc_state;  // 内存分配状态
    uint32_t size;              // 单个缓冲区大小（字节）
    _Buffer_t buffers[2];           // 两个缓冲区（0和1）
    uint8_t last_writed_idx;        // 最新写入完成的缓冲区索引（0/1）
} DoubleBuffer_t;

/// @brief 创建并初始化双缓冲区
/// @param db 双缓冲区实例指针
/// @param size 单个缓冲区大小（字节）
/// @return 操作结果
DoubleBuffer_Return_e DoubleBuffer_Create(DoubleBuffer_t *db, uint32_t size);

/// @brief 写入数据到双缓冲区（按状态表逻辑选择目标）
/// @param db 双缓冲区实例指针
/// @param data 待写入的数据（长度需与buf_size一致）
/// @return 操作结果
DoubleBuffer_Return_e DoubleBuffer_Write(DoubleBuffer_t *db, const void *data);

/// @brief 从双缓冲区读取数据（优先读取最新写入的缓冲区）
/// @param db 双缓冲区实例指针
/// @param data 接收数据的缓冲区（长度需与buf_size一致）
/// @return 操作结果
DoubleBuffer_Return_e DoubleBuffer_Read(DoubleBuffer_t *db, void *data);

/// @brief 销毁双缓冲区，释放内存
/// @param db 双缓冲区实例指针
void DoubleBuffer_Delete(DoubleBuffer_t *db);

#endif // DOUBLE_BUFFER_H
