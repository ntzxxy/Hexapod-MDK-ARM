#include "cmsis_os.h"
#include "lcd.h"
#include "mpu6050.h" // MPU6050 角度数据的头文件
#include <cstdio>    // 使用 C++ 标准库中的 snprintf
#include "LegControl_task.h"

extern MPU_6050 mpu6050; // 引用全局的 MPU6050 对象

extern "C" { 

void LCD_Task(void const * argument)
{
    char buffer[20];
    
    // [1] LCD 初始化 (现在是正确的公共函数调用，解决链接错误)
    LCD_Init(); 
    osDelay(1000);
		
    while (1)
    {
				const char* mode_str;
        switch(current_mode) {
            case MODE_IDLE:         mode_str = "IDLE"; break;
            case MODE_IK_TEST:      mode_str = "IK"; break;
            case MODE_RESET_ZERO:   mode_str = "RESET"; break;
            case MODE_SINGLE_DEBUG: mode_str = "DEBUG"; break;
            default:                mode_str = "UNKNOWN"; break;
        }
				std::snprintf(buffer, sizeof(buffer), "P:%6.2f", mpu6050.angle.x);
        LCD_ShowString(4, 4, ST7735Ctx.Width, 16, 16, (uint8_t*)buffer); 

        // --- 刷新 Roll ---
				std::snprintf(buffer, sizeof(buffer), "R:%6.2f", mpu6050.angle.y);
        LCD_ShowString(4, 22, ST7735Ctx.Width, 16, 16, (uint8_t*)buffer);
        
        // --- 刷新 Yaw ---
				std::snprintf(buffer, sizeof(buffer), "Y:%6.2f", mpu6050.angle.z);
				LCD_ShowString(4, 40, ST7735Ctx.Width, 16, 16, (uint8_t*)buffer);
				
				std::snprintf(buffer, sizeof(buffer), "Mode:%s", mode_str);
        LCD_ShowString(30, 4, ST7735Ctx.Width, 16, 16, (uint8_t*)buffer);
				
				std::snprintf(buffer, sizeof(buffer), "Tgt-Z:%6.1f", target_pos.z);
        LCD_ShowString(30, 22, ST7735Ctx.Width, 16, 16, (uint8_t*)buffer);
				
			
				//sprintf((char *)&text, "WeAct Studio");
				//LCD_ShowString(4, 4, ST7735Ctx.Width, 16, 16, text);
				//sprintf((char *)&text, "STM32H7xx 0x%X", HAL_GetDEVID());
				//LCD_ShowString(4, 22, ST7735Ctx.Width, 16, 16, text);
				//sprintf((char *)&text, "LCD ID:0x%X", st7735_id);
				//LCD_ShowString(4, 40, ST7735Ctx.Width, 16, 16, text);
				
        osDelay(300); // 100ms 刷新一次
    }
}

} 
