#ifndef KEY_TASK_H
#define KEY_TASK_H

#include "cmsis_os.h"

// 声明任务函数（方便其他地方调用，虽然freertos.c不需要）
#ifdef __cplusplus
extern "C" {
#endif

void Key_Task(void const * argument);

#ifdef __cplusplus
}
#endif

// 声明队列句柄，这样包含这个头文件的文件都能用队列
extern osMessageQId KeyQueueHandle;

#endif