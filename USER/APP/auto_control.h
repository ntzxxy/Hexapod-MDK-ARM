#ifndef __AUTO_CONTROL_H
#define __AUTO_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

typedef struct {
    uint8_t mode_sw;		// 1:Wave, 2:调姿, 3:Tripod
    uint8_t gait_sw;		// 0:普通, 1:越障(高抬腿)
    int8_t  vx_raw;			// 前后速度 (预设20)
    int8_t  vy_raw;			// 左右速度
    int8_t  w_raw;			// 旋转速度 (Omega)
    int8_t body_h;			// 高度
    uint32_t last_tick;
} AutoControl_t;

// 供外部调用的接口
void AutoControl_Init(UART_HandleTypeDef *huart);
void Process_Raw_Data(uint8_t *data, uint16_t len);
void AutoControl_Report(void);


extern AutoControl_t auto_data;
extern uint8_t rx_buf[40];

#ifdef __cplusplus
}
#endif

#endif
