#include "cmsis_os.h"
#include "main.h"    // 包含了常用的 HAL 宏和类型
#include "usart.h"   // 包含了所有 huart 句柄的声明
#include "gpio.h"
#include "bsp.h"     // 包含了 TX/RX 使能宏
#include "leg.h"     // 包含了 Leg 类
#include "my_math.h" // 包含了 PI 常量和 Thetas 结构体
#include "dwt_delay_us.h"
#include "Servo.h"   // 包含舵机指令宏
#include "gait_prg.h"
#include "debug_uart.h"

// ------------------------------------------------------------------
// --- 1. 最小结构体定义 (代替复杂的 Hexapod 类) ---
// ------------------------------------------------------------------

uint32_t LegControl_round; // 机器人回合数
//Hexapod hexapod;		   // 机器人结构体

Gait_prg gait_prg;	  // 步态规划
uint32_t round_time;  // 回合时间
Velocity test_velocity;       // 模拟机器人速度输入
Thetas leg_offset[6]; // 腿部关节角偏移，用于将舵机相对机器人本体的角度换算至相对舵机本身的角度
// 任务中需要使用的最小化 Leg 结构体实例 (仅代表第一条腿)
Leg my_leg;
Thetas test_thetas; // 用于设置角度的最小数据结构

// 声明外部 UART 句柄 (假设我们调试 Leg 0，使用 huart1)
extern UART_HandleTypeDef huart1;
extern Thetas ikine(Position3 &pos);

// ------------------------------------------------------------------
// --- 2. 初始化函数 (替代 Hexapod::Init) ---
// ------------------------------------------------------------------

/**
 * @brief 初始化单个 Leg 对象（只初始化 LEG1/USART1）
 */
void Minimal_Test_Init(void)
{
    // C++ 对象构造：将 Leg 对象与 USART1 句柄关联
    my_leg = Leg(&huart1);

		gait_prg.Init();
	
		test_velocity.Vx = 30.0f; // 假设 30 mm/s 前进速度
    test_velocity.Vy = 0.0f;
    test_velocity.omega = 0.0f;
    gait_prg.set_velocity(test_velocity); // 将速度设置给步态规划器
    // 设置初始角度 (所有舵机归零)
    test_thetas = Thetas(0.0f, 0.0f, 0.0f);
    my_leg.set_thetas(test_thetas);
    
    // 设置初始移动时间 (500ms)
    my_leg.set_time(500);

    // 初始化 DWT (用于微秒级延时)
    DWT_Delay_Init();
}

// ------------------------------------------------------------------
// --- 3. FreeRTOS 任务入口函数 ---
// ------------------------------------------------------------------

/**
 * @brief FreeRTOS 任务入口函数 (临时替换原 LegControl_Task)
 */
extern "C"
{
	void LegControl_Task(void const *argument)
	{
		Minimal_Test_Init(); // 初始化 Leg 对象

		osDelay(500); // 启动延时，让系统稳定
        
        //float target_angle = PI / 4.0f; // 初始目标角度 45度
				//float target_angle2 = PI / 6.0f; // 初始目标角度 45度
				//float target_angle3 = PI / 2.0f; // 初始目标角度 45度
        //uint16_t move_time = 500; 
		
				// 定义腿末端目标位置（相对于腿起始端，单位mm）
        Position3 target_pos(150.0f, 0.0f, -100.0f); 
        
        // 存储逆解结果
        Thetas solved_thetas;

		while (1)
		{
            // 切换 0度 和 45度 之间往复运动
            //target_angle = (target_angle > 0.1f) ? 0.0f : (PI / 4.0f); 
						//target_angle2 = (target_angle2 > 0.1f) ? 0.0f : (PI / 6.0f); 
						//target_angle3 = (target_angle3 > 0.1f) ? 0.0f : (PI / 2.0f); 
            //my_leg.set_time(move_time);
            
            // =========================================================================
            // 模式选择：取消注释您要测试的模式 (请务必确保只选择一种模式)
            // =========================================================================
            
            // --- 模式 A: 测试单个舵机 (Servo 0, ID 1) ---
            
            // 1. 仅设置第一个舵机 Coxa 的角度
            //my_leg.get_servo(0).set_angle(target_angle);
            
            // 2. 调用阻塞式单舵机发送函数 (您需要在 leg.cpp 中实现 move_single_servo_blocking_test)
            //my_leg.move_single_servo_blocking_test(0); 
            
            
            // --- 模式 B: 测试整条腿 (三个舵机: Servo 0, 1, 2) ---
						
            // 1. 设置三个舵机的安全角度 (例如：Coxa, Femur 45度, Tibia -45度)
            //test_thetas.angle[0] = target_angle; // 舵机 1
            //test_thetas.angle[1] = target_angle2; // 舵机 2
            //test_thetas.angle[2] = -target_angle3; // 舵机 3
            //my_leg.set_thetas(test_thetas); // 会调用 set_angle for all 3 servos
            
            // 2. 调用阻塞式三舵机发送函数 (原 leg.cpp 中已存在，使用 HAL_UART_Transmit)
            //my_leg.move_UART(); 
						
            // =========================================================================


						solved_thetas = ikine(target_pos);
            
            // ------------------------------------------------
            // 步骤 2: 打印解算结果（将弧度转换为角度方便验证）
            // ------------------------------------------------
            //APP_PRINT("IKINE Solved Angles:\r\n");
            APP_PRINT("  C%.2f\r\n", solved_thetas.angle[0] * 180.0f / PI);
						osDelay(1000);
            APP_PRINT("  F%.2f\r\n", solved_thetas.angle[1] * 180.0f / PI);
						osDelay(1000);
            APP_PRINT("  T%.2f\r\n", solved_thetas.angle[2] * 180.0f / PI);
						osDelay(1000);

            // ------------------------------------------------
            // 步骤 3: 验证是否在安全范围内
            // ------------------------------------------------
            // 舵机角度范围大致在 -120度到120度之间
            if (solved_thetas.angle[0] == 0.0f && solved_thetas.angle[1] == 0.0f && solved_thetas.angle[2] == 0.0f)
            {
                APP_PRINT("WARNING\r\n");
            }
			osDelay(5000); // 延时 1000ms 等待舵机完成动作
		}
	}
}

