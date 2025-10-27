#include "double_buffer.h"
#include <stdlib.h>
#include <string.h>

DoubleBuffer_Return_e DoubleBuffer_Create(DoubleBuffer_t *double_buffer, uint32_t size)
{
  if (double_buffer == NULL || size == 0) {
    return DOUBLE_BUFFER_RETURN_NULLPTR;
  }
  
  double_buffer->malloc_state = DOUBLE_BUFFER_MALLOC_NONE;
  double_buffer->size = size;
  double_buffer->last_writed_idx = 1; // initialize to 0/1 is both OK
  double_buffer->buffers[0].state = _BUFFER_STATE_IDLE;
  double_buffer->buffers[1].state = _BUFFER_STATE_IDLE;

  // alloc memory to 2 buffers, must keep the same size
  double_buffer->buffers[0].buf = calloc(size, 1);
  if (double_buffer->buffers[0].buf == NULL) {
    return DOUBLE_BUFFER_RETURN_NOMEM;
  }
  double_buffer->buffers[1].buf = calloc(size, 1);
  if (double_buffer->buffers[1].buf == NULL) {
    free(double_buffer->buffers[0].buf);
    return DOUBLE_BUFFER_RETURN_NOMEM;
  }
  double_buffer->malloc_state = DOUBLE_BUFFER_MALLOC_DONE;
  return DOUBLE_BUFFER_RETURN_SUCCESS;
}

void DoubleBuffer_Delete(DoubleBuffer_t *double_buffer)
{
  if (double_buffer == NULL) {
    return;
  }
  if (double_buffer->malloc_state == DOUBLE_BUFFER_MALLOC_DONE) {
    free(double_buffer->buffers[0].buf);
    free(double_buffer->buffers[1].buf);
  }
}

// read/write logic: (lst : last writed_buffer, !lst : another buffer)
// lst            !lst          write_to    read_from
// IDLE           IDLE          !lst        NONE
// IDLE           READING       lst         NONE
// IDLE           WRITING       lst         NONE
// IDLE           WRITED        lst         !lst
// READING        IDLE          !lst        NONE
// READING        READING       NONE        NONE
// READING        WRITING       NONE        NONE
// READING        WRITED        !lst        NONE
// WRITING        IDLE          !lst        NONE
// WRITING        READING       NONE        NONE
// WRITING        WRITING       NONE        NONE
// WRITING        WRITED        !lst        NONE
// WRITED         IDLE          !lst        lst
// WRITED         READING       lst         lst
// WRITED         WRITING       lst         lst
// WRITED         WRITED        !lst        lst

// 写入目标映射表[last_writed_state][!last_writed_state] = write_to_idx（-1表示无目标）
// 严格对应表格"write_to"列
static const int8_t write_table[4][4] = {
    {1, 0, 0, 0},   // lst=IDLE(0)：对应表格行1-4
    {1, -1, -1, 1}, // lst=READING(1)：对应表格行5-8
    {1, -1, -1, 1}, // lst=WRITING(2)：对应表格行9-12
    {1, 0, 0, 1}    // lst=WRITED(3)：对应表格行13-16
};

// 读取目标映射表[last_writed_state][!last_writed_state] = read_from_idx（-1表示无目标）
// 严格对应表格"read_from"列
static const int8_t read_table[4][4] = {
    {-1, -1, -1, 1}, // lst=IDLE(0)：对应表格行1-4
    {-1, -1, -1, -1}, // lst=READING(1)：对应表格行5-8
    {-1, -1, -1, -1}, // lst=WRITING(2)：对应表格行9-12
    {0, 0, 0, 0}     // lst=WRITED(3)：对应表格行13-16
};

DoubleBuffer_Return_e DoubleBuffer_Write(DoubleBuffer_t *db, const void *data)
{
    // 参数校验
    if (db == NULL || data == NULL) {
        return DOUBLE_BUFFER_RETURN_NULLPTR;
    }
    if (db->malloc_state != DOUBLE_BUFFER_MALLOC_DONE) {
        return DOUBLE_BUFFER_RETURN_NOMEM;
    }

    // 获取当前状态和索引
    uint8_t last_writed_idx = db->last_writed_idx; // 最新写入的索引
    uint8_t another_idx = !last_writed_idx; // 非运算取另一个索引
    _Buffer_State_e last_writed_state = db->buffers[last_writed_idx].state;
    _Buffer_State_e other_state = db->buffers[another_idx].state;

    // 查表获取写入目标（状态转换为0-3的索引）
    int8_t write_to_idx = write_table[(int)last_writed_state][(int)other_state];
    if (write_to_idx < 0 || write_to_idx > 1) { // 无效索引（-1）
        return DOUBLE_BUFFER_RETURN_NOTARGET;
    }

    // 执行写入（状态锁定→复制数据→更新状态）
    db->buffers[write_to_idx].state = _BUFFER_STATE_WRITING; // 禁止读取
    memcpy(db->buffers[write_to_idx].buf, data, db->size);   // 复制数据
    db->buffers[write_to_idx].state = _BUFFER_STATE_WRITED;  // 标记为可读取
    db->last_writed_idx = write_to_idx;                      // 更新最新索引

    return DOUBLE_BUFFER_RETURN_SUCCESS;
}

DoubleBuffer_Return_e DoubleBuffer_Read(DoubleBuffer_t *db, void *data)
{
    // 参数校验
    if (db == NULL || data == NULL) {
        return DOUBLE_BUFFER_RETURN_NULLPTR;
    }
    if (db->malloc_state != DOUBLE_BUFFER_MALLOC_DONE) {
        return DOUBLE_BUFFER_RETURN_NOMEM;
    }

    // 获取当前状态和索引
    uint8_t last_writed_idx = db->last_writed_idx; // 最新写入的索引
    uint8_t another_idx = !last_writed_idx; // 非运算取另一个索引
    _Buffer_State_e last_writed_state = db->buffers[last_writed_idx].state;
    _Buffer_State_e other_state = db->buffers[another_idx].state;

    // 查表获取读取目标（状态转换为0-3的索引）
    int8_t read_from_idx = read_table[(int)last_writed_state][(int)other_state];
    if (read_from_idx < 0 || read_from_idx > 1) { // 无效索引（-1）
        return DOUBLE_BUFFER_RETURN_NOTARGET;
    }

    // 执行读取（状态锁定→复制数据→更新状态）
    db->buffers[read_from_idx].state = _BUFFER_STATE_READING; // 禁止写入
    memcpy(data, db->buffers[read_from_idx].buf, db->size);   // 复制数据
    db->buffers[read_from_idx].state = _BUFFER_STATE_IDLE;    // 标记为可写入

    return DOUBLE_BUFFER_RETURN_SUCCESS;
}
