#ifndef IMAGE_QUEUE_TASK_H
#define IMAGE_QUEUE_TASK_H

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include <stdint.h>

// 图像缓冲区配置
#define IMAGE_WIDTH 96
#define IMAGE_HEIGHT 240
#define IMAGE_SIZE (IMAGE_WIDTH * IMAGE_HEIGHT)
#define IMAGE_BUFFER_SIZE 24576
#define MAX_QUEUE_SIZE 10// 最大队列深度为5张图像

// 静态缓冲区结构
typedef struct {
    uint8_t data[IMAGE_BUFFER_SIZE];  // 图像数据
    uint16_t size;                    // 实际数据大小
    uint8_t valid;                    // 数据有效标志
    uint8_t inUse;                    // 使用中标志
} StaticImageFrame_t;

// 队列项（只存索引，节省队列内存）
typedef struct {
    uint8_t bufferIndex;  // 缓冲区索引
    uint16_t size;        // 数据大小
} QueueItem_t;

// 图像队列控制结构（简化版本，去掉状态管理）
typedef struct {
    QueueHandle_t imageQueue;       // FreeRTOS队列句柄
    SemaphoreHandle_t queueMutex;   // 队列保护互斥锁
    uint32_t totalReceived;         // 总接收图像数
    uint32_t totalProcessed;        // 总处理图像数
    uint32_t droppedFrames;         // 丢弃的帧数
} ImageQueueControl_t;




HAL_StatusTypeDef ImageQueue_Init(void);  // 改为返回状态
void ImageQueue_Task(void *argument);


// 零拷贝相关接口
int8_t ImageQueue_AllocateBuffer(void);  // 分配空闲缓冲区，返回索引
HAL_StatusTypeDef ImageQueue_CommitBuffer(int8_t bufferIndex, uint16_t size);  // 提交缓冲区到队列
HAL_StatusTypeDef ImageQueue_ReleaseBuffer(int8_t bufferIndex);  // 释放缓冲区
uint8_t* ImageQueue_GetBufferPtr(int8_t bufferIndex);  // 获取缓冲区指针

uint8_t ImageQueue_GetStatus(void);
HAL_StatusTypeDef ImageQueue_Start(void);

HAL_StatusTypeDef ImageQueue_DeInit(void);  


#endif /* IMAGE_QUEUE_TASK_H */