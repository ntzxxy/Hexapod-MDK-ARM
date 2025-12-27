#ifndef REMOTE_H
#define REMOTE_H

#include "main.h"


/***********改串口号的话要改这里****************/
#define HAL_REMOTE_UART_Init MX_UART7_Init
#define REMOTE_UART UART7
#define REMOTE_UART_h huart7
/************************************************/


#define SBUS_RX_BUF_LEN 64
#define REMOTE_DATA_LEN 25  //一帧18字节
#ifdef __cplusplus
extern "C"{
#endif
typedef struct
{
	int16_t right_HRZC;  //右边水平摇杆
	int16_t right_VETC;	//右边垂直摇杆
	int16_t left_HRZC;		//左边水平摇杆
	int16_t left_VETC;		//左边垂直摇杆
	int16_t mode_sw;    // CH8 (SWD): 主模式 (Lock/Adjust/Walk) - 替代原来的 S1
  int16_t mpu_sw;     // CH6 (SWB): MPU开关 - 替代原来的 S2
  int16_t gait_sw;    // CH7 (SWC): 步态切换
  int16_t func_sw;    // CH5 (SWA): 回中开关，用于触发重置
	int16_t knob_VRA;   // CH9
  int16_t knob_VRB;   // CH10
	
}RC_remote_data_t;


void Remote_Init(void);
RC_remote_data_t Remote_read_data(void);
void Remote_UART_Callback(UART_HandleTypeDef *huart);

extern uint32_t remote_hock;

#ifdef __cplusplus
}
#endif
	
#endif
