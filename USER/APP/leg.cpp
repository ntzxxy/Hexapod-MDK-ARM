#include "leg.h"
#include "gait_prg.h"
#include "stdlib.h"
#include "cmsis_os.h"
#include "main.h"
#include "usart.h"
#include "bsp.h"
#include "debug_uart.h"


uint8_t receive_buffer[100];

Leg::Leg(UART_HandleTypeDef *huart)
{
	this->huart = huart;
	servos[0] = Servo(1);
	servos[1] = Servo(2);
	servos[2] = Servo(3);
}

void Leg::set_cal_offset(Thetas offset) 
{ 
	this->cal_offset = offset; 
}

void Leg::set_thetas(Thetas theta)
{
	this->theta.angle[0] = theta.angle[0];
	this->theta.angle[1] = theta.angle[1];
	this->theta.angle[2] = theta.angle[2];
	// 舵机 0 (Coxa): IK角度 + 机械校准偏移
	float angle_0_cmd = theta.angle[0] + this->cal_offset.angle[0];
  // 舵机 1 (Femur): IK角度 + 机械校准偏移 (+5.85度)
	float angle_1_cmd = theta.angle[1] + this->cal_offset.angle[1];
  // 舵机 2 (Tibia): IK角度 + 机械校准偏移 (+3.47度) - Tibia中心偏移 (-85度)
	float angle_2_cmd = theta.angle[2] + this->cal_offset.angle[2] - TIBIA_CENTER_OFFSET;
	//串口调试信息
	if (this->huart->Instance == USART1) 
    {
        // 将弧度转换为角度，方便肉眼判断
        float deg0 = theta.angle[0] * 180.0f / PI;
        float deg1 = theta.angle[1] * 180.0f / PI;
        float deg2 = theta.angle[2] * 180.0f / PI;

        // 打印格式：[目标] J0(Coxa), J1(Femur), J2(Tibia)
        APP_PRINT("TGT: %.2f, %.2f, %.2f\r\n", deg0, deg1, deg2);
    }
	
	this->servos[0].set_angle(angle_0_cmd);
	this->servos[1].set_angle(angle_1_cmd);
	this->servos[2].set_angle(angle_2_cmd);
}

void Leg::set_time(uint16_t move_time)
{
	servos[0].set_time(move_time);
	servos[1].set_time(move_time);
	servos[2].set_time(move_time);
}

void Leg::move_DMA()
{
	this->TX_Enable();
	this->servos[0].move(this->send_buffer);
	this->servos[1].move(this->send_buffer + SERVO_MOVE_TIME_WRITE_LEN + 3);
	this->servos[2].move(this->send_buffer + (SERVO_MOVE_TIME_WRITE_LEN + 3) * 2);
	HAL_UART_Transmit_DMA(this->huart, this->send_buffer, (SERVO_MOVE_TIME_WRITE_LEN + 3) * 3);
}

void Leg::move_UART()
{
	this->TX_Enable();
	this->servos[0].move(this->send_buffer);
	this->servos[1].move(this->send_buffer + SERVO_MOVE_TIME_WRITE_LEN + 3);
	this->servos[2].move(this->send_buffer + (SERVO_MOVE_TIME_WRITE_LEN + 3) * 2);
	HAL_UART_Transmit(this->huart, this->send_buffer, (SERVO_MOVE_TIME_WRITE_LEN + 3) * 3, 1000);
}

void Leg::move_wait()
{
	this->TX_Enable();
	this->servos[0].move_wait(this->send_buffer);
	this->servos[1].move_wait(this->send_buffer + SERVO_MOVE_TIME_WAIT_WRITE_LEN + 3);
	this->servos[2].move(this->send_buffer + (SERVO_MOVE_TIME_WAIT_WRITE_LEN + 3) * 2);
	HAL_UART_Transmit_DMA(this->huart, this->send_buffer, (SERVO_MOVE_TIME_WAIT_WRITE_LEN + 3) * 3);
}

void Leg::move_start()
{
	this->TX_Enable();
	this->servo_broad_cast.move_start(this->send_buffer);
	HAL_UART_Transmit_DMA(this->huart, this->send_buffer, SERVO_MOVE_START_LEN + 3);
}

void Leg::load()
{
	this->servo_broad_cast.load(this->send_buffer);
	HAL_UART_Transmit_DMA(this->huart, this->send_buffer, SERVO_LOAD_OR_UNLOAD_WRITE_LEN + 3);
}

void Leg::unload()
{
	this->servo_broad_cast.unload(this->send_buffer);
	HAL_UART_Transmit_DMA(this->huart, this->send_buffer, SERVO_LOAD_OR_UNLOAD_WRITE_LEN + 3);
}

void Leg::read_angle(uint32_t id)
{
	this->TX_Enable();
	this->servos[id - 1].read_angle(this->send_buffer);
	HAL_UART_Transmit_DMA(this->huart, this->send_buffer, SERVO_POS_READ_LEN + 3); // 为了在发送完成后再开启接收，这里不使用DMA
	while (__HAL_UART_GET_FLAG(this->huart, UART_FLAG_TC))
		; // 等待串口发送完成

	this->RX_Enable(); // 使能接收
	HAL_UART_Receive(this->huart, receive_buffer, RECV_SERVO_POS_READ_LEN + 3, 1000);
	float angle = (((uint16_t)receive_buffer[5] | ((uint16_t)receive_buffer[6] << 8)) - 500) / 750 * PI;
	this->theta.angle[id - 1] = angle;
	this->servos[id - 1].set_angle(angle);

	this->TX_Enable(); // 发送完毕后使能发送
}

// 阻塞式单舵机移动测试
void Leg::move_single_servo_blocking_test(uint8_t servo_index)
{
    // 安全检查
    if (servo_index > 2) 
        return;

    // 1. 准备单个舵机的命令数据包
    // 将指定舵机的命令写入 send_buffer 的起始位置
    this->servos[servo_index].move(this->send_buffer);

    // 2. 启用发送，禁用接收（半双工切换）
    this->TX_Enable();

    // 3. 阻塞发送数据包
    // SERVO_MOVE_TIME_WRITE_LEN (7) + 3 (头和ID/长度) = 10 字节
    uint16_t packet_len = SERVO_MOVE_TIME_WRITE_LEN + 3; 
    
    // 使用 HAL_UART_Transmit (阻塞模式)
    // 超时时间 1000ms
    HAL_UART_Transmit(this->huart, this->send_buffer, packet_len, 1000);

    // 4. 禁用发送，使能接收（切换回接收模式，准备接收或等待下一命令）
    this->RX_Enable(); 
}

// 使能接收
void Leg::RX_Enable()
{
	switch ((uint32_t)(this->huart->Instance))
	{
	case (uint32_t)USART1:
		__LEG1_RXEN();
		break;
	case (uint32_t)USART2:
		__LEG2_RXEN();
		break;
	case (uint32_t)USART3:
		__LEG3_RXEN();
		break;
	case (uint32_t)UART4:
		__LEG4_RXEN();
		break;
	case (uint32_t)UART5:
		__LEG5_RXEN();
		break;
	case (uint32_t)USART6:
		__LEG6_RXEN();
		break;
	default:
		break;
	}
}

// 使能发送
void Leg::TX_Enable()
{
	switch ((uint32_t)(this->huart->Instance))
	{
	case (uint32_t)USART1:
		__LEG1_TXEN();
		break;
	case (uint32_t)USART2:
		__LEG2_TXEN();
		break;
	case (uint32_t)USART3:
		__LEG3_TXEN();
		break;
	case (uint32_t)UART4:
		__LEG4_TXEN();
		break;
	case (uint32_t)UART5:
		__LEG5_TXEN();
		break;
	case (uint32_t)USART6:
		__LEG6_TXEN();
		break;
	default:
		break;
	}
}

void Leg::TX_Unable()
{
	switch ((uint32_t)(this->huart->Instance))
	{
	case (uint32_t)USART1:
		__LEG1_TXUEN();
		break;
	case (uint32_t)USART2:
		__LEG2_TXUEN();
		break;
	case (uint32_t)USART3:
		__LEG3_TXUEN();
		break;
	case (uint32_t)UART4:
		__LEG4_TXUEN();
		break;
	case (uint32_t)UART5:
		__LEG5_TXUEN();
		break;
	case (uint32_t)USART6:
		__LEG6_TXUEN();
		break;
	default:
		break;
	}
}
