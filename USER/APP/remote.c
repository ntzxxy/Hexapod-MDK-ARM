#include "remote.h"
#include "usart.h"
#include "debug_uart.h"
#include "string.h"

RC_remote_data_t rc_remote_data;
uint32_t remote_hock;
uint8_t msg;
static uint8_t sbus_rx_buf[SBUS_RX_BUF_LEN];
void SBUS_Parse_Frame(uint8_t *frame);
void SBUS_Parse_Stream(uint8_t *buf, uint16_t len);

static uint8_t SBUS_Switch_Map(int16_t val)
{
	if (val > 1500) return 3;      // 下
	else if (val < 500) return 1;  // 上 
	else return 2;                 // 中 
}

//初始化
void Remote_Init()
{
	
	memset(sbus_rx_buf, 0, SBUS_RX_BUF_LEN);
	
		__HAL_UART_CLEAR_OREFLAG(&huart7); 
    __HAL_UART_CLEAR_NEFLAG(&huart7);  
    __HAL_UART_CLEAR_FEFLAG(&huart7);  
    __HAL_UART_CLEAR_PEFLAG(&huart7);

	// 使用中断接收 SBUS 的 25 字节
		HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(&REMOTE_UART_h, sbus_rx_buf, SBUS_RX_BUF_LEN);
if (status != HAL_OK) {
    // 完蛋，DMA 启动失败！通常是因为 DMA 通道被占用或者没初始化
    APP_PRINT("DMA Start Failed: %d\r\n", status);
}
}



//读遥控数据
RC_remote_data_t Remote_read_data()
{
	//因为数据可能在处理的过程中被DMA覆盖，故需要拷贝一份出来
	static RC_remote_data_t remote_data_buffer;
	static RC_remote_data_t zero_data; //if remote data is vaild ,return zero data
	remote_data_buffer = rc_remote_data;
	
	//数据预处理
	remote_data_buffer.left_HRZC -=992;
	remote_data_buffer.left_VETC -=992;
	remote_data_buffer.right_HRZC -=992;
	remote_data_buffer.right_VETC -=992;
	remote_data_buffer.knob_VRA    -= 992;
	remote_data_buffer.knob_VRB    -= 992;
	//check if the data is vaild
	if(remote_data_buffer.left_HRZC > 800 || remote_data_buffer.left_HRZC < -800 ||
	   remote_data_buffer.left_VETC > 800 || remote_data_buffer.left_VETC < -800 ||
	   remote_data_buffer.right_HRZC > 800 || remote_data_buffer.right_HRZC < -800 ||
	   remote_data_buffer.right_VETC > 800 || remote_data_buffer.right_VETC < -800
	  )
	{
		// 数据异常，尝试重启串口接收
		// HAL_UART_DeInit(&REMOTE_UART_h); // 频繁 DeInit 可能会导致卡死，建议只重置接收
		// Remote_Init();
		
		// 也可以简单地清零结构体并返回
		memset(&rc_remote_data, 0, sizeof(RC_remote_data_t));
		// 喂狗复位，认为失联
		return zero_data;
	}
	//死区处理
	if(remote_data_buffer.left_HRZC>-25&&remote_data_buffer.left_HRZC<25)
		remote_data_buffer.left_HRZC = 0;
	if(remote_data_buffer.left_VETC>-25&&remote_data_buffer.left_VETC<25)
		remote_data_buffer.left_VETC = 0;
	if(remote_data_buffer.right_HRZC>-25&&remote_data_buffer.right_HRZC<25)
		remote_data_buffer.right_HRZC = 0;
	if(remote_data_buffer.right_VETC>-25&&remote_data_buffer.right_VETC<25)
		remote_data_buffer.right_VETC = 0;
	
	
	return remote_data_buffer;
}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == UART7)
    {
        SBUS_Parse_Stream(sbus_rx_buf, Size);

        HAL_UARTEx_ReceiveToIdle_DMA(&huart7,
                                     sbus_rx_buf,
                                     SBUS_RX_BUF_LEN);
    }
}

void SBUS_Parse_Stream(uint8_t *buf, uint16_t len)
{
    for (int i = 0; i + 24 < len; i++)
    {
        if (buf[i] == 0x0F && buf[i + 24] == 0x00)
        {
            SBUS_Parse_Frame(&buf[i]);
            return;
        }
    }
}

void SBUS_Parse_Frame(uint8_t *frame)
{
	
			// 判断 SBUS 帧头 (0x0F) 和 帧尾 (0x00)
		if (frame[0] == 0x0F && frame[24] == 0x00)
			{
			// [SBUS 解析] 11位数据解析逻辑
			// 临时变量
			int16_t ch[16];

			ch[0]  = ((int16_t)frame[1] >> 0 | ((int16_t)frame[2] << 8 )) & 0x07FF;
			ch[1]  = ((int16_t)frame[2] >> 3 | ((int16_t)frame[3] << 5 )) & 0x07FF;
			ch[2]  = ((int16_t)frame[3] >> 6 | ((int16_t)frame[4] << 2 ) | (int16_t)frame[5] << 10 ) & 0x07FF;
			ch[3]  = ((int16_t)frame[5] >> 1 | ((int16_t)frame[6] << 7 )) & 0x07FF;
			ch[4]  = ((int16_t)frame[6] >> 4 | ((int16_t)frame[7] << 4 )) & 0x07FF;
			ch[5]  = ((int16_t)frame[7] >> 7 | ((int16_t)frame[8] << 1 ) | (int16_t)frame[9] <<  9 ) & 0x07FF;
			ch[6]  = ((int16_t)frame[9] >> 2 | ((int16_t)frame[10] << 6 )) & 0x07FF;
			ch[7]  = ((int16_t)frame[10] >> 5 | ((int16_t)frame[11] << 3 )) & 0x07FF;
			ch[8]  = ((int16_t)frame[12] << 0 | ((int16_t)frame[13] << 8 )) & 0x07FF;
			ch[9]  = ((int16_t)frame[13] >> 3 | ((int16_t)frame[14] << 5 )) & 0x07FF;
			
			// [通道映射]
			// 塔克 HT-10A 默认通道顺序：1:Roll, 2:Pitch, 3:Throttle, 4:Yaw
			// 注意：如果发现控制方向不对，请在这里交换 ch[x]
			rc_remote_data.right_HRZC = ch[0]; // Ch1 副翼
			rc_remote_data.right_VETC = ch[1]; // Ch2 升降
			rc_remote_data.left_VETC  = ch[2]; // Ch3 油门
			rc_remote_data.left_HRZC  = ch[3]; // Ch4 方向
			
			// [开关映射]
			// 将 SBUS 的模拟值映射为 1, 2, 3 状态
			rc_remote_data.func_sw = SBUS_Switch_Map(ch[4]); // Ch5 -> S1
			rc_remote_data.mpu_sw = (ch[5] > 1000) ? 0 : 1;
			
			//rc_remote_data.func_sw = ch[4]; // Ch5 -> S1
			//rc_remote_data.mpu_sw = ch[5];
			rc_remote_data.gait_sw = (ch[6] > 1000) ? 0 : 1;
			//rc_remote_data.gait_sw = ch[6];
			
			rc_remote_data.mode_sw = SBUS_Switch_Map(ch[7]);
			//rc_remote_data.mode_sw = ch[7];

			// [新增] 旋钮映射 (Ch9, Ch10) -> 用于步态切换和高度
			rc_remote_data.knob_VRA = ch[8];
			rc_remote_data.knob_VRB = ch[9];
			
			remote_hock = 0; // 喂狗，表示通信正常
			}
	
}

