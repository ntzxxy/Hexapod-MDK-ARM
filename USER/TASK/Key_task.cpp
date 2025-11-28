/**
  * @file Key_task.cpp
  * @brief 独立的按键扫描任务 (驱动层)
  */
#include "cmsis_os.h"
#include "main.h"   
#include "key.h"    // 包含 Independent_Key_Scan()
#include "debug_uart.h" 

// 引用外部的消息队列句柄 (定义在 LegControl_task.cpp 中)
extern osMessageQId KeyQueueHandle;

// 任务入口函数
extern "C" {

void Key_Task(void const * argument)
{
    
    Key key_driver;
    

    uint8_t current_key = 0;
    static uint8_t last_key = 0;
    static uint32_t press_start_tick = 0;

    for(;;)
    {
        // 1. 调用你原来的底层函数读取原始键值
        current_key = key_driver.Scan();

        // 2. 状态机消抖逻辑
        if (current_key != 0) 
        {
            if (last_key == 0) 
            {
                // 刚按下，记录时刻
                press_start_tick = osKernelSysTick(); 
            }
            else if (last_key == current_key) 
            {
                // 持续按下，检查是否超过 20ms (消抖)
                if ((osKernelSysTick() - press_start_tick) > 20) 
                {
                    // --- 确认有效按下，发送消息到应用层 ---
                    if (KeyQueueHandle != NULL) 
                    {
                        // 发送键值，超时时间设为 0 (非阻塞，发不出去就算了，别卡死按键扫描)
                        osMessagePut(KeyQueueHandle, current_key, 0);
                        
                        // 【等待释放】防止按一下发送多次
                        // 这里使用阻塞等待，因为这是独立任务，不会卡死 LegControl 任务
                        while(key_driver.Scan() == current_key) 
                        {
                            osDelay(10);
                        }
                    }
                }
            }
            last_key = current_key; // 更新上一次状态
        }
        else 
        {
            last_key = 0; // 按键抬起，重置状态
        }

        // 3. 扫描频率 (10ms 检测一次足够了)
        osDelay(10); 
    }
}

}