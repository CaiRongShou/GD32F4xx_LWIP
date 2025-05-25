#ifndef __RING_BUFFER_H
#define __RING_BUFFER_H

#include <stdint.h>

/**
 * @brief 环形缓冲区数据结构
 */
typedef struct {
    volatile uint8_t* buffer;  /**< 缓冲区 */
    volatile uint16_t size;    /**< 缓冲区大小 */
    volatile uint16_t write_index; /**< 写入索引 */
    volatile uint16_t read_index;  /**< 读取索引 */
    volatile uint16_t mirror;  /**< 镜像位 */
} ring_buffer_t;

/**
 * @brief 初始化环形缓冲区
 * @param rb 指向环形缓冲区的指针
 * @param buffer 缓冲区
 * @param size 缓冲区大小
 */
void ring_buffer_init(ring_buffer_t* rb, uint8_t* buffer, uint16_t size);

/**
 * @brief 从环形缓冲区中读取数据
 * @param rb 指向环形缓冲区的指针
 * @param buffer 用于存储读取数据的缓冲区
 * @param size 要读取的字节数
 * @return 已读取的字节数
 */
uint16_t ring_buffer_read(ring_buffer_t* rb, uint8_t* buffer, uint16_t size);

/**
 * @brief 向环形缓冲区中写入数据
 * @param rb 指向环形缓冲区的指针
 * @param data 要写入的数据
 * @param size 要写入的字节数
 * @return 已写入的字节数
 */
uint16_t ring_buffer_write(ring_buffer_t* rb, const uint8_t* data, uint16_t size);

/**
 * @brief 获取环形缓冲区中当前待读取的数据大小
 * @param rb 指向环形缓冲区的指针
 * @return 待读取的数据大小（字节数）
 */
uint16_t ring_buffer_available_read(const ring_buffer_t* rb);

/**
 * @brief 重置环形缓冲区
 * @param rb 指向环形缓冲区的指针
 */
void ring_buffer_reset(ring_buffer_t* rb);

#endif // __RING_BUFFER_H
