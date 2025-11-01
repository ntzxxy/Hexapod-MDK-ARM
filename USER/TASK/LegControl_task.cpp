#include "cmsis_os.h"
#include "main.h"
#include "usart.h" // 包含 CubeMX 生成的 UART 句柄
#include "gpio.h"
#include "bsp.h" // 包含 TX/RX 使能宏
#include "leg.h" // 包含 Leg 类
#include "my_math.h" // 包含 PI 常量和 Thetas 结构体
#include "dwt_delay_us.h"

// ------------------------------------------------------------------
// --- 1. 最小结构体定义 (代替复杂的 Hexapod 类) ---
// ------------------------------------------------------------------

// 任务中需要使用的最小化 Leg 结构体实例
Leg my_leg; 
Thetas initial_thetas; // 用于设置角度的最小数据结构

// 声明外部 UART 句柄 (请确保与 CubeMX 配置一致)
// 假设舵机使用 USART1 (根据您的引脚配置 PB14/15)
extern UART_HandleTypeDef huart1; 

// ------------------------------------------------------------------
// --- 2. 最小化初始化函数 (替代 Hexapod::Init) ---
// ------------------------------------------------------------------

/**
 * @brief 初始化单个 Leg 对象（只初始化 LEG1/USART1）
 */
void Minimal_Hexapod_Init(void)
{
    // C++ 对象构造：将 Leg 对象与 USART1 句柄关联
    my_leg = Leg(&huart1);
    
    // 假设我们只使用 Coxa (舵机 1) 进行控制
    // 初始角度设置为 0 (弧度)
    initial_thetas.angle[0] = 0.0f; // Coxa 0度
    initial_thetas.angle[1] = 0.0f; // Femur 0度
    initial_thetas.angle[2] = 0.0f; // Tibia 0度
    
    // 设置初始角度 (Leg::set_thetas 内部会调用 3 个 Servo::set_angle)
    my_leg.set_thetas(initial_thetas); 
    
    // 设置初始移动时间
    my_leg.set_time(500); 

    // 初始化 DWT (如果需要微秒级延时，必须调用)
    DWT_Delay_Init(); 
    
    // 注意：MX_USART1_UART_Init() 等函数由 main() 调用
}

// ------------------------------------------------------------------
// --- 3. FreeRTOS 任务入口函数 (最小化替换) ---
// ------------------------------------------------------------------

/**
 * @brief FreeRTOS 任务入口函数 (使用 extern "C" 解决链接问题)
 */
extern "C"
{
	void LegControl_Task(void const *argument)
	{
		Minimal_Hexapod_Init(); // 初始化 Leg 对象

		osDelay(500); // 启动延时，让系统稳定

		float target_angle = PI / 2.0f; // 初始目标角度 90度

		while (1)
		{
            // ------------------------------------------------
            // 步骤 1: 设置目标角度 (仅 Coxa 舵机)
            // ------------------------------------------------
            target_angle = (target_angle > 0.0f) ? 0.0f : (PI / 2.0f); // 切换 0度 和 90度
            
            // 将角度设置到 Leg 结构中
            initial_thetas.angle[0] = target_angle; 
            my_leg.set_thetas(initial_thetas); 
            
            // ------------------------------------------------
            // 步骤 2: 发送指令 (调用 Leg::move_DMA)
            // ------------------------------------------------
            // Leg::move_DMA 内部会调用 TX_Enable 和 HAL_UART_Transmit_DMA
            my_leg.move_DMA();

            // ------------------------------------------------
            // 步骤 3: 等待舵机完成动作并接收 DMA 中断完成信号
            // ------------------------------------------------
            // 任务在此阻塞，留出时间让舵机移动 (500ms) 和 DMA 发送完成
			osDelay(1000); 
		}
	}
}

// ------------------------------------------------------------------
// --- 4. C 任务所需的 C++ 成员函数实现 (必须移植) ---
// ------------------------------------------------------------------
// 注意：以下函数体必须存在于工程中，请确保 leg.cpp 和 Servo.cpp 已加入工程。
// 例如：
// * Leg::Leg(UART_HandleTypeDef *huart)
// * Leg::set_thetas()
// * Leg::move_DMA()
// * Leg::TX_Enable()/RX_Enable()
// * Servo::set_angle()
// * Servo::move()



