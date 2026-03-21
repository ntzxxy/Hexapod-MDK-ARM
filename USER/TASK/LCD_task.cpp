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
		UIMode_t last_ui_mode = UI_MODE_MAX;
		
    while (1)
    {
				if (current_ui_mode != last_ui_mode)
        {
            // 用黑色填充整个屏幕
            // 注意：FillRect 函数名可能因库而异，请确认是 FillRect 还是 LCD_Fill
            ST7735_LCD_Driver.FillRect(&st7735_pObj, 0, 0, ST7735Ctx.Width, ST7735Ctx.Height, BLACK);
            
            // 更新 last_mode，防止下一轮循环继续清屏
            last_ui_mode = current_ui_mode;
        }
				
				switch(current_ui_mode) 
        {
            // === 界面 1: 运动数据显示 ===
            case UI_MODE_MOVEMENT:
            {
                // 1. 标题
                LCD_ShowString(4, 4, ST7735Ctx.Width, 16, 16, (uint8_t*)"RUN MODE");
                
                // 2. 显示当前状态机状态
                const char* state_str;
                switch(current_mode) {
                    case MODE_IDLE: state_str = "IDLE"; break;
                    case MODE_GAIT_RUN: state_str = "WALK"; break;
                    default: state_str = "---"; break;
                }
                std::snprintf(buffer, sizeof(buffer), "%s", state_str);
                LCD_ShowString(4, 22, ST7735Ctx.Width, 16, 16, (uint8_t*)buffer);

                // 3. 显示 MPU 数据 (姿态)
								std::snprintf(buffer, sizeof(buffer), "P%4.1f", mpu6050.angle.x);
								LCD_ShowString(0, 40, ST7735Ctx.Width, 16, 16, (uint8_t*)buffer); 

								// --- 刷新 Roll ---
								std::snprintf(buffer, sizeof(buffer), "R%4.1f", mpu6050.angle.y);
								LCD_ShowString(50, 40, ST7735Ctx.Width, 16, 16, (uint8_t*)buffer);
								
								// --- 刷新 Yaw ---
								std::snprintf(buffer, sizeof(buffer), "Y%4.1f", mpu6050.angle.z);
								LCD_ShowString(100, 40, ST7735Ctx.Width, 16, 16, (uint8_t*)buffer);
								
								Thetas leg1_rad = hexapod.legs[0].get_current_thetas();
				
								float ang0 = leg1_rad.angle[0] * 180.0f / PI;
								float ang1 = leg1_rad.angle[1] * 180.0f / PI;
								float ang2 = leg1_rad.angle[2] * 180.0f / PI;
								
								std::snprintf(buffer, sizeof(buffer), "L1:%3.0f %3.0f %4.0f", ang0, ang1, ang2);
								LCD_ShowString(4, 58, ST7735Ctx.Width, 16, 16, (uint8_t*)buffer);

                
                break;
            }

            // === 界面 2: 示教模式显示 ===
            case UI_MODE_TEACHING:
            {
                LCD_ShowString(4, 4, ST7735Ctx.Width, 16, 16, (uint8_t*)"TEACH MODE");

                // 2. 提示当前舵机状态
                if (g_motor_power_on) 
								{
										// 上电状态显示 ON (绿色或者普通颜色)
										LCD_ShowString(4, 22, ST7735Ctx.Width, 16, 16, (uint8_t*)"PWR:ON");
								}
								else 
								{
										// 掉电状态显示 OFF
										LCD_ShowString(4, 22, ST7735Ctx.Width, 16, 16, (uint8_t*)"PWR:OFF");
								}
                
                // 3. 显示一条腿的角度作为参考
               Thetas leg0_rad = hexapod.legs[0].get_current_thetas();
				
								float ang0 = leg0_rad.angle[0] * 180.0f / PI;
								float ang1 = leg0_rad.angle[1] * 180.0f / PI;
								float ang2 = leg0_rad.angle[2] * 180.0f / PI;
								
								std::snprintf(buffer, sizeof(buffer), "L1:%3.0f %3.0f %4.0f", ang0, ang1, ang2);
								LCD_ShowString(4, 40, ST7735Ctx.Width, 16, 16, (uint8_t*)buffer);
								
								const char* state_str;
                switch(current_mode) {
                    case MODE_IDLE: state_str = "IDLE"; break;
                    case MODE_GAIT_RUN: state_str = "WALK"; break;
                    default: state_str = "---"; break;
                }
                std::snprintf(buffer, sizeof(buffer), "%s", state_str);
                LCD_ShowString(4, 58, ST7735Ctx.Width, 16, 16, (uint8_t*)buffer);
               
                break;
            }
            
            // === 界面 3: 调试显示 ===
            case UI_MODE_DEBUG:
            {
                 LCD_ShowString(4, 4, ST7735Ctx.Width, 16, 16, (uint8_t*)"DEBUG IK");
                 std::snprintf(buffer, sizeof(buffer), "Leg%d", CALIB_LEG_INDEX);
                 LCD_ShowString(4, 22, ST7735Ctx.Width, 16, 16, (uint8_t*)buffer);
                 std::snprintf(buffer, sizeof(buffer), "X:%.0f Y:%.0f", calib_target_pos.x, calib_target_pos.y);
								 LCD_ShowString(0, 40, ST7735Ctx.Width, 16, 16, (uint8_t*)buffer);

								 std::snprintf(buffer, sizeof(buffer), "Z:%.1f", calib_target_pos.z);
								 LCD_ShowString(0, 58, ST7735Ctx.Width, 16, 16, (uint8_t*)buffer);
									
                 break;
            }
						 case UI_MODE_REMOTE:
            {
                // 1. 标题
                LCD_ShowString(4, 4, ST7735Ctx.Width, 16, 16, (uint8_t*)"REMOTE");
                
                // 2. 显示当前状态机状态
                const char* state_str;
                switch(current_mode) {
                    case MODE_IDLE: state_str = "IDLE"; break;
                    case MODE_GAIT_RUN: state_str = "WALK"; break;
                    default: state_str = "---"; break;
                }
                std::snprintf(buffer, sizeof(buffer), "%s", state_str);
                LCD_ShowString(4, 22, ST7735Ctx.Width, 16, 16, (uint8_t*)buffer);

                // 3. 显示 MPU 数据 (姿态)
								std::snprintf(buffer, sizeof(buffer), "P%4.1f", mpu6050.angle.x);
								LCD_ShowString(0, 40, ST7735Ctx.Width, 16, 16, (uint8_t*)buffer); 

								// --- 刷新 Roll ---
								std::snprintf(buffer, sizeof(buffer), "R%4.1f", mpu6050.angle.y);
								LCD_ShowString(50, 40, ST7735Ctx.Width, 16, 16, (uint8_t*)buffer);
								
								// --- 刷新 Yaw ---
								std::snprintf(buffer, sizeof(buffer), "ROU%d", LegControl_round);
								LCD_ShowString(100, 40, ST7735Ctx.Width, 16, 16, (uint8_t*)buffer);
								
								Thetas leg1_rad = hexapod.legs[0].get_current_thetas();
				
								float ang0 = leg1_rad.angle[0] * 180.0f / PI;
								float ang1 = leg1_rad.angle[1] * 180.0f / PI;
								float ang2 = leg1_rad.angle[2] * 180.0f / PI;
								
								std::snprintf(buffer, sizeof(buffer), "L1:%3.0f %3.0f %4.0f", ang0, ang1, ang2);
								LCD_ShowString(4, 58, ST7735Ctx.Width, 16, 16, (uint8_t*)buffer);

                
                break;
            }
						case UI_MODE_BALANCE:
            {
                // 1. 标题
                LCD_ShowString(4, 4, ST7735Ctx.Width, 16, 16, (uint8_t*)"BALANCE");
                
                // 2. 显示当前状态机状态
                const char* state_str;
                switch(current_mode) {
                    case MODE_IDLE: state_str = "IDLE"; break;
                    case MODE_BALANCE_TEST: state_str = "BALT"; break;
                    default: state_str = "---"; break;
                }
                std::snprintf(buffer, sizeof(buffer), "%s", state_str);
                LCD_ShowString(4, 22, ST7735Ctx.Width, 16, 16, (uint8_t*)buffer);

                // 3. 显示 MPU 数据 (姿态)
								std::snprintf(buffer, sizeof(buffer), "P%4.1f", mpu6050.angle.x);
								LCD_ShowString(0, 40, ST7735Ctx.Width, 16, 16, (uint8_t*)buffer); 

								// --- 刷新 Roll ---
								std::snprintf(buffer, sizeof(buffer), "R%4.1f", mpu6050.angle.y);
								LCD_ShowString(50, 40, ST7735Ctx.Width, 16, 16, (uint8_t*)buffer);
								
								// --- 刷新 Yaw ---
								std::snprintf(buffer, sizeof(buffer), "Y%4.1f", mpu6050.angle.z);
								LCD_ShowString(100, 40, ST7735Ctx.Width, 16, 16, (uint8_t*)buffer);
								
								Thetas leg1_rad = hexapod.legs[0].get_current_thetas();
				
								float ang0 = leg1_rad.angle[0] * 180.0f / PI;
								float ang1 = leg1_rad.angle[1] * 180.0f / PI;
								float ang2 = leg1_rad.angle[2] * 180.0f / PI;
								
								std::snprintf(buffer, sizeof(buffer), "L1:%3.0f %3.0f %4.0f", ang0, ang1, ang2);
								LCD_ShowString(4, 58, ST7735Ctx.Width, 16, 16, (uint8_t*)buffer);

                
                break;
            }
						
						case UI_MODE_SHOW:
            {
                // 1. 标题
                LCD_ShowString(4, 4, ST7735Ctx.Width, 16, 16, (uint8_t*)"SHOW");
                 
								
								Thetas leg1_rad = hexapod.legs[0].get_current_thetas();
				
								float ang0 = leg1_rad.angle[0] * 180.0f / PI;
								float ang1 = leg1_rad.angle[1] * 180.0f / PI;
								float ang2 = leg1_rad.angle[2] * 180.0f / PI;
								
								std::snprintf(buffer, sizeof(buffer), "L1:%3.0f %3.0f %4.0f", ang0, ang1, ang2);
								LCD_ShowString(4, 58, ST7735Ctx.Width, 16, 16, (uint8_t*)buffer);

                
                break;
            }
						
						 case UI_MODE_AUTO:
            {
                // 1. 标题
                LCD_ShowString(4, 4, ST7735Ctx.Width, 16, 16, (uint8_t*)"AUTO");
                
                // 2. 显示当前状态机状态
                const char* state_str;
                switch(current_mode) {
                    case MODE_IDLE: state_str = "IDLE"; break;
                    case MODE_GAIT_RUN: state_str = "WALK"; break;
                    default: state_str = "---"; break;
                }
                std::snprintf(buffer, sizeof(buffer), "%s", state_str);
                LCD_ShowString(4, 22, ST7735Ctx.Width, 16, 16, (uint8_t*)buffer);

                // 3. 显示 MPU 数据 (姿态)
								std::snprintf(buffer, sizeof(buffer), "P%4.1f", mpu6050.angle.x);
								LCD_ShowString(0, 40, ST7735Ctx.Width, 16, 16, (uint8_t*)buffer); 

								// --- 刷新 Roll ---
								std::snprintf(buffer, sizeof(buffer), "R%4.1f", mpu6050.angle.y);
								LCD_ShowString(50, 40, ST7735Ctx.Width, 16, 16, (uint8_t*)buffer);
								
								// --- 刷新 Yaw ---
								std::snprintf(buffer, sizeof(buffer), "ROU%d", LegControl_round);
								LCD_ShowString(100, 40, ST7735Ctx.Width, 16, 16, (uint8_t*)buffer);
								
								Thetas leg1_rad = hexapod.legs[0].get_current_thetas();
				
								float ang0 = leg1_rad.angle[0] * 180.0f / PI;
								float ang1 = leg1_rad.angle[1] * 180.0f / PI;
								float ang2 = leg1_rad.angle[2] * 180.0f / PI;
								
								std::snprintf(buffer, sizeof(buffer), "L1:%3.0f %3.0f %4.0f", ang0, ang1, ang2);
								LCD_ShowString(4, 58, ST7735Ctx.Width, 16, 16, (uint8_t*)buffer);

                
                break;
            }
        }

        osDelay(300); // 刷新频率
  			
		}
} 
}
