#ifndef LEG_H
#define LEG_H

#include "Servo.h"
#include "usart.h"
#include "my_math.h"

class Leg
{
private:
	uint8_t send_buffer[100]; //发送缓存
	Servo servos[3];
	Thetas theta;
	Servo_Broad_Cast servo_broad_cast;
	UART_HandleTypeDef *huart;
	Thetas cal_offset;
	uint8_t leg_id;
	public:
	Leg(UART_HandleTypeDef *huart, uint8_t leg_id, uint8_t start_servo_id); // 构造函数
	Leg(){};						// 无参构造
	Thetas get_current_thetas() 
  { 
      return this->theta; 
  }
	void TX_Enable();
	void RX_Enable();
	void TX_Unable();
	void set_thetas(Thetas thetas); // 设置机械腿的角度
	void set_cal_offset(Thetas offset);
	void set_time(uint16_t tims);	// 设置机械腿移动的时间
	void move_DMA();					// 机械腿移动命令
	void move_UART();
	void move_wait();				// 设置机械腿角度，但需要等待开始命令才移动
	void move_start();				// 让机械腿开始运动
	void load();					// 上电
	void unload();					// 掉电
	void read_angle(uint32_t id);	// 读取舵机角度
	void move_single_servo_blocking_test(uint8_t servo_index);	// 阻塞式单舵机移动测试
	void prepare_move_buffer();
	uint8_t* get_send_buffer() 
	{ 
		return this->send_buffer; 
	}
	Servo& get_servo(uint8_t index) 
    {
        return this->servos[index];
    }
};

#endif
