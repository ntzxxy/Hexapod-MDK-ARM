#include "auto_control.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>

AutoControl_t auto_data;
// 缓冲区稍微加大，建议为帧长的2倍左右，确保滑窗时不会漏掉跨包的帧
uint8_t rx_buf[40]; 

static volatile uint16_t auto_ok_cnt = 0;   // 成功计数
static uint32_t last_report_tick = 0;       // 上次发送时间
static volatile uint8_t auto_flag = 0;

void AutoControl_Init(UART_HandleTypeDef *huart) {
    memset(rx_buf, 0, sizeof(rx_buf));
		memset(&auto_data, 0, sizeof(auto_data));
		auto_ok_cnt = 0;
    last_report_tick = 0;
    auto_flag = 0;
    auto_data.last_tick = 0;

    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_PEFLAG(huart);

    HAL_UARTEx_ReceiveToIdle_DMA(huart, rx_buf, sizeof(rx_buf));

    // 可选：关闭半传输中断，减少无用中断
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
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
				auto_flag = 1;
				if (auto_ok_cnt == 0xFFFF)
				{
					auto_ok_cnt = 0;
				}
				else {
					auto_ok_cnt++;
				}
				

    } else {
        //HAL_UART_Transmit(&huart5, (uint8_t*)"CS ERR\r\n", 8, 10);
    }
}

void AutoControl_Report(void)
{
    uint32_t now = HAL_GetTick();
	
		if (auto_flag == 0) {
        return;
    }
		if ((now - auto_data.last_tick) > 300) {
        auto_flag = 0;
        auto_ok_cnt = 0;          // 重新连接后重新计数
        last_report_tick = now;   // 防止恢复后立刻补发
        return;
    }
    if (now - last_report_tick >= 2000)  // 2秒
    {
        last_report_tick = now;

        char buf[32];
				
        //int len = snprintf(buf, sizeof(buf), "%u\r\n", auto_ok_cnt);
        //HAL_UART_Transmit(&huart5, (uint8_t*)buf, len, 50);
				
    }
}

void Process_Raw_Data(uint8_t *buf, uint16_t len) {
    // 滑动窗口：i是窗口起始位置，i+8是帧尾位置
    for (int i = 0; i + 8 < len; i++) {
        // 寻找你的协议特征：头0xAA 尾0x55
        if (buf[i] == 0xAA && buf[i + 8] == 0x55) {
            AutoControl_Process_Frame(&buf[i]);
            // 找到一帧并处理后，直接结束本次扫描，防止重复处理
            return; 
        }
    }
    // 如果循环结束还没找到，说明这一包里没有完整合法的帧
    // 这里可以不做操作，等待下一波数据填入
}