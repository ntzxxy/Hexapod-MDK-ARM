#include "auto_control.h"
#include "usart.h"
#include <string.h>

AutoControl_t auto_data;
// 缓冲区稍微加大，建议为帧长的2倍左右，确保滑窗时不会漏掉跨包的帧
uint8_t rx_buf[20]; 

void AutoControl_Init(UART_HandleTypeDef *huart) {
    memset(rx_buf, 0, sizeof(rx_buf));
    // 开启 DMA 空闲中断接收
    HAL_UART_Receive_DMA(huart, rx_buf, sizeof(rx_buf));
}

// 仿照 SBUS_Parse_Frame：只负责解析固定长度的 9 字节
void AutoControl_Process_Frame(uint8_t *frame) {
    uint8_t sum = 0;
    for (int j = 0; j < 7; j++) sum += frame[j];

    if (sum == frame[7]) {
        // 校验通过，更新数据
        auto_data.mode_sw = frame[1];
        auto_data.gait_sw = frame[2];
        auto_data.vx_raw  = (int8_t)frame[3];
        auto_data.vy_raw  = (int8_t)frame[4];
        auto_data.w_raw   = (int8_t)frame[5];
        auto_data.body_h  = frame[6];
        auto_data.last_tick = HAL_GetTick();

    } else {
        //HAL_UART_Transmit(&huart5, (uint8_t*)"CS ERR\r\n", 8, 10);
    }
}

// 仿照 SBUS_Parse_Stream：滑窗寻找帧头 0xAA 和 帧尾 0x55
void Process_Raw_Data(uint8_t *buf, uint16_t len) {
    uint8_t frame_found = 0;

    // 如果长度连一帧都不到，直接退出，不报 SYNC ERR，等下次接够了再说
    if (len < 9) return;

    // 滑动窗口寻找
    for (int i = 0; i <= (len - 9); i++) {
        if (buf[i] == 0xAA && buf[i + 8] == 0x55) {
								AutoControl_Process_Frame(&buf[i]);
                frame_found = 1;
                break; // 只要找到一帧有效的，就跳出循环，防止处理残留数据
            }
        }
		
    if (!frame_found) {
        // 只有真的翻遍了都没有 AA...55 才报错
        HAL_UART_Transmit(&huart5, (uint8_t*)"SYNC ERR\r\n", 10, 10);
    }
		
}