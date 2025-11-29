#include "LegControl_task.h"
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
#include "mpu6050.h"


using namespace std;

extern osMessageQId KeyQueueHandle;

ControlMode_t current_mode = MODE_IDLE; // 当前模式，默认空闲

uint32_t LegControl_round; // 机器人回合数
Hexapod hexapod;		   // 机器人结构体
bool is_dirty = false;

Gait_prg gait_prg;	  // 步态规划
uint32_t round_time;  // 回合时间
Velocity test_velocity;       // 模拟机器人速度输入(测试结束请删除)
Velocity velocity;
Thetas leg_offset[6]; // 腿部关节角偏移，用于将舵机相对机器人本体的角度换算至相对舵机本身的角度
// 任务中需要使用的最小化 Leg 结构体实例 (仅代表第一条腿)
Leg my_leg;
Thetas test_thetas; // 用于设置角度的最小数据结构

// 定义腿末端目标位置（相对于腿起始端，单位mm）
Position3 target_pos; 

extern UART_HandleTypeDef huart2;
extern Thetas ikine(Position3 &pos);

void key_deal(void);

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
		Thetas calibration_offset(
        0.0f,                                   // Coxa (J0): 0度
        11.85f * PI / 180.0f,                    // Femur (J1): +5.85度校准 (弧度)
        7.47f * PI / 180.0f                     // Tibia (J2): +3.47度校准 (弧度)
    ); 

    my_leg.set_cal_offset(calibration_offset); // <<< 关键：应用校准偏移
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
		
		//Minimal_Test_Init(); // 初始化 Leg 对象
		hexapod.Init();
		gait_prg.Init();
		Thetas leg1_cal_offset(
            0.0f,                                   
            11.85f * PI / 180.0f,                    
            7.47f * PI / 180.0f                     
        );
		hexapod.legs[1].set_cal_offset(leg1_cal_offset);
		DWT_Delay_Init();
		osDelay(300); // 启动延时，让系统稳定
		
		
		static uint32_t code_time_start, code_time_end, code_time; // 用于计算程序运行时间，保证程序隔一段时间跑一遍
	      
    target_pos = Position3(170.0f, 0.0f, -100.0f);  
    uint16_t move_time = 500; 
		
		uint32_t last_move_tick = 0;
    bool motion_toggle = false; // 用于往复运动的标志位

		while (1)
		{
						code_time_start = xTaskGetTickCount();
						key_deal();						
						switch (current_mode)
            {
                case MODE_IDLE:
                    // 空闲状态，什么都不做，或者发送软力矩指令
                    break;

                case MODE_IK_TEST:
                    // 每隔 2000ms 执行一次往复运动 (替代原来的 osDelay(5000))
                    if (HAL_GetTick() - last_move_tick > 2000)
                    {
                        last_move_tick = HAL_GetTick();
                        
                        // 切换目标位置（简单的往复测试）
                        motion_toggle = !motion_toggle;
                        if(motion_toggle) {
                            target_pos.z = -100.0f; // 下去
                        } else {
                            target_pos.z = -50.0f;  // 上来
                        }
												is_dirty = true;
										}
                    break;
								case MODE_GAIT_RUN:
										gait_prg.set_velocity(hexapod.velocity);
										if (hexapod.velocity.omega >= 0)
												LegControl_round = (++LegControl_round) % N_POINTS; // 控制回合自增长
										else
										{
												if (LegControl_round == 0)
												LegControl_round = N_POINTS - 1;
												else
												LegControl_round--;
										}
										/*步态控制*/
										gait_prg.CEN_and_pace_cal();
										gait_prg.gait_proggraming();
										/*开始移动*/
										round_time = gait_prg.get_pace_time() / N_POINTS;
										hexapod.move(round_time);
										// 计算程序运行时间
										code_time_end = xTaskGetTickCount();		 // 获取当前systick时间
										code_time = code_time_end - code_time_start; // 做差获取程序运行时间（8ms）
										if (code_time < round_time)
												osDelay(round_time - code_time); // 保证程序执行周期等于回合时间
										else
												osDelay(1); // 至少延时1ms
								continue;
                case MODE_RESET_ZERO:
                    // 归零逻辑...
                    break;
            }
            // 切换 0度 和 45度 之间往复运动
						
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
            //test_thetas.angle[2] = target_angle3; // 舵机 3
            //my_leg.set_thetas(test_thetas); // 会调用 set_angle for all 3 servos
            
            // 2. 调用阻塞式三舵机发送函数 (原 leg.cpp 中已存在，使用 HAL_UART_Transmit)
           // my_leg.move_UART(); 
						
            // =========================================================================


						//solved_thetas = ikine(target_pos);
            
            // ------------------------------------------------
            // 步骤 2: 打印解算结果（将弧度转换为角度方便验证）
            // ------------------------------------------------
            //APP_PRINT("IKINE Solved Angles:\r\n");
            //APP_PRINT("  C%.2f\r\n", solved_thetas.angle[0] * 180.0f / PI);
						//osDelay(1000);
            //APP_PRINT("  F%.2f\r\n", solved_thetas.angle[1] * 180.0f / PI);
						//osDelay(1000);
            //APP_PRINT("  T%.2f\r\n", solved_thetas.angle[2] * 180.0f / PI);
						//osDelay(1000);
						
						//my_leg.set_thetas(solved_thetas);
						//my_leg.move_UART();
            // ------------------------------------------------
            // 步骤 3: 验证是否在安全范围内
            // ------------------------------------------------
            // 舵机角度范围大致在 -120度到120度之间
            //if (solved_thetas.angle[0] == 0.0f && solved_thetas.angle[1] == 0.0f && solved_thetas.angle[2] == 0.0f)
            //{
                //APP_PRINT("WARNING\r\n");
            //}
						if (is_dirty) {
								Thetas solved = ikine(target_pos);
								my_leg.set_thetas(solved);
								my_leg.move_DMA(); // 推荐使用 DMA
								is_dirty = false; // 清除标志，等待下次改变
						}
			osDelay(20); // 延时 1000ms 等待舵机完成动作
		}
	}
}

void key_deal(void)
{
    osEvent event = osMessageGet(KeyQueueHandle, 0); 
    if (event.status == osEventMessage)
    {
        uint8_t key_val = (uint8_t)event.value.v;

        switch (key_val)
        {
            case 1: // 按键1: 启动原地踏步/行走
                current_mode = MODE_GAIT_RUN;
                // 模拟遥控器输入：原地
                hexapod.velocity.Vx = 0;
                hexapod.velocity.Vy = 0;
                hexapod.velocity.omega = 0;
                break;
            case 2: // 按键2: 前进
                current_mode = MODE_GAIT_RUN;
                hexapod.velocity.Vx = 0.0f;
								hexapod.velocity.Vy = 30.0f;
                break;
            case 3: // Key3: 进入单腿调试模式
                current_mode = MODE_IK_TEST;
                break;
						case 4: // Key4: 停止
                current_mode = MODE_IDLE;
                hexapod.velocity.Vx = 0;
                break;
        }
    }
}


// 初始化腿部变量并初始化串口，使能串口发送
void Hexapod::Init(void)
{
	legs[0] = Leg(&huart1);
	legs[1] = Leg(&huart2);
	legs[2] = Leg(&huart3);
	legs[3] = Leg(&huart4);
	legs[4] = Leg(&huart5);
	legs[5] = Leg(&huart6);
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
	MX_UART4_Init();
	MX_UART5_Init();
	MX_USART6_UART_Init();
	leg_offset[0] = Thetas(PI / 4, LEG_JOINT2_OFFSET, LEG_JOINT3_OFFSET);
	leg_offset[1] = Thetas(0.0f, LEG_JOINT2_OFFSET, LEG_JOINT3_OFFSET);
	leg_offset[2] = Thetas(-PI / 4, LEG_JOINT2_OFFSET, LEG_JOINT3_OFFSET);
	leg_offset[3] = Thetas(3 * PI / 4, LEG_JOINT2_OFFSET, LEG_JOINT3_OFFSET);
	leg_offset[4] = Thetas(PI, LEG_JOINT2_OFFSET, LEG_JOINT3_OFFSET);
	leg_offset[5] = Thetas(-3 * PI / 4, LEG_JOINT2_OFFSET, LEG_JOINT3_OFFSET);
	mpu_pid_x.Init(MPU_X_PID_KP, MPU_X_PID_KI, MPU_X_PID_KD, CIR_OFF);
	mpu_pid_y.Init(MPU_Y_PID_KP, MPU_Y_PID_KI, MPU_Y_PID_KD, CIR_OFF);
	velocity_fof[0].set_k_filter(VELOCITY_FOF_K);
	velocity_fof[1].set_k_filter(VELOCITY_FOF_K);
	velocity_fof[2].set_k_filter(VELOCITY_FOF_K);
	body_pos_fof[0].set_k_filter(BODY_POS_FOF_K);
	body_pos_fof[1].set_k_filter(BODY_POS_FOF_K);
	body_pos_fof[2].set_k_filter(BODY_POS_FOF_K);
	body_angle_fof[0].set_k_filter(BODY_ANGLE_FOF_K);
	body_angle_fof[1].set_k_filter(BODY_ANGLE_FOF_K);
	body_angle_fof[2].set_k_filter(BODY_ANGLE_FOF_K);
}

// 计算机器人速度
void Hexapod::velocity_cal(const RC_remote_data_t &remote_data)
{
	if (this->mode != HEXAPOD_MOVE) // 若不是行走模式则速度为0
	{
		velocity.Vx = 0;
		velocity.Vy = 0;
		velocity.omega = 0;
	}
	else
	{
		velocity.Vx = velocity_fof[0].cal(0.3f * remote_data.right_HRZC);
		velocity.Vy = velocity_fof[1].cal(0.3f * remote_data.right_VETC);
		velocity.omega = velocity_fof[2].cal(-0.3f * remote_data.left_HRZC);
	}
	if(velocity.Vx>-0.0001f && velocity.Vx<0.0001f)velocity.Vx=0;
	if(velocity.Vy>-0.0001f && velocity.Vy<0.0001f)velocity.Vy=0;
	if(velocity.omega>-0.0001f && velocity.omega<0.0001f)velocity.omega=0;
	gait_prg.set_velocity(velocity);
}

void Hexapod::body_position_cal(const RC_remote_data_t &remote_data)
{
	if (this->mode != HEXAPOD_BODY_ANGEL_CONTROL) // 除了姿态控制模式，其他情况下都能控制z轴高度
		body_pos.z += ROTATE_BODY_POS_SENSI * remote_data.left_VETC;
	if (this->mode == HEXAPOD_BODY_POS_CONTROL) // 若是身体位置控制模式则计算xy位置
	{
		// body_pos.y += ROTATE_BODY_POS_SENSI * remote_data.right_VETC;
		// body_pos.x += ROTATE_BODY_POS_SENSI * remote_data.right_HRZC;
		body_pos.y = HEXAPOD_MAX_Y/660.0f * remote_data.right_VETC;
		body_pos.x = -HEXAPOD_MIN_X/660.0f * remote_data.right_HRZC;
	}   
	//限制数值
	value_limit(body_pos.z, HEXAPOD_MIN_HEIGHT, HEXAPOD_MAX_HEIGHT);
	value_limit(body_pos.y, HEXAPOD_MIN_Y, HEXAPOD_MAX_Y);
	value_limit(body_pos.x, HEXAPOD_MIN_X, HEXAPOD_MAX_X);
	//一阶低通滤波
	body_pos.y = body_angle_fof[1].cal(body_pos.y);
	body_pos.x = body_angle_fof[2].cal(body_pos.x);
	gait_prg.set_body_position(body_pos);
}

void Hexapod::body_angle_cal(const RC_remote_data_t &remote_data)
{
	mpu_angle = mpu6050.get_angle();			  // 获取陀螺仪角度
	if (this->mode != HEXAPOD_BODY_ANGEL_CONTROL) // 若不是姿态控制模式则直接返回
		return;
	if (this->mpu_flag == false && this->mpu_sw == MPU_ON) // 第一次进入陀螺仪模式
	{
		this->mpu_angle_set = this->mpu_angle; // 将设定角度等于陀螺仪角度
		this->mpu_flag = true;				   // 标志位置1
	}
	if (this->mpu_sw == MPU_OFF) // 不是陀螺仪模式则标志位置零
		this->mpu_flag = false;
	if (this->mpu_sw == MPU_ON)
	{
		mpu_angle_set.x -= ROTATE_BODY_ANGLE_SENSI * remote_data.left_VETC;
		mpu_angle_set.y += ROTATE_BODY_ANGLE_SENSI * remote_data.left_HRZC;
		//mpu_angle_set.z += ROTATE_BODY_ANGLE_SENSI * remote_data.right_HRZC;
		// body_angle.x = this->mpu_angle.x;
		// body_angle.y = this->mpu_angle.y;
		// float mpu_x_add = mpu_angle_set.x - mpu_angle.x;
		// float mpu_y_add = mpu_angle_set.y - mpu_angle.y;
		body_angle.x += this->mpu_pid_x.cal(mpu_angle.x, mpu_angle_set.x);
		body_angle.y -= this->mpu_pid_y.cal(mpu_angle.y, mpu_angle_set.y);
		// body_angle.x = this->mpu_angle.x + this->mpu_pid_x.cal(mpu_angle.x, mpu_angle_set.x);
		// body_angle.y = this->mpu_angle.y - this->mpu_pid_y.cal(mpu_angle.y, mpu_angle_set.y);
		// body_angle.z += (mpu_angle_set.z - mpu_angle.z); //陀螺仪z轴零漂严重，暂时不使用
	}
	else
	{
		body_angle.x = body_angle_fof[0].cal(-0.001f * remote_data.left_VETC);
		body_angle.y = body_angle_fof[1].cal(-0.001f * remote_data.left_HRZC);
		body_angle.z = body_angle_fof[2].cal(0.001f * remote_data.right_HRZC);
	} 
	value_limit(body_angle.x, HEXAPOD_MIN_X_ROTATE, HEXAPOD_MAX_X_ROTATE);
	value_limit(body_angle.y, HEXAPOD_MIN_Y_ROTATE, HEXAPOD_MAX_Y_ROTATE);
	value_limit(body_angle.z, HEXAPOD_MIN_Z_ROTATE, HEXAPOD_MAX_Z_ROTATE);


	gait_prg.set_body_rotate_angle(body_angle);
}

/*
 *@brief 检查拨轮数据，符合要求就归零
 *@param remote_data 遥控数据
 */
void Hexapod::body_angle_and_pos_zero(const RC_remote_data_t &remote_data)
{
	if (remote_data.thumb_wheel > 500)
	{
		this->body_angle.zero();
		this->body_pos.zero();
		this->mpu_angle_set.zero();
	}
}

void Hexapod::mode_select(const RC_remote_data_t &remote_data)
{
	switch (remote_data.S1)
	{
	case 1:					 // 拨杆在上面
		mode = HEXAPOD_MOVE; // 移动模式
		break;
	case 2:								   // 拨杆在下面
		mode = HEXAPOD_BODY_ANGEL_CONTROL; // 机身旋转角度控制
		break;
	case 3:								 // 拨杆在中间
		mode = HEXAPOD_BODY_POS_CONTROL; // 机身位置控制
	default:
		break;
	}
	switch (remote_data.S2)
	{
	case 0:
		mpu_sw = MPU_ON;
		break;
	case 1:
		mpu_sw = MPU_OFF;
		break;
	default:
		break;
	}
}

static void remote_deal(void)
{
	static RC_remote_data_t remote_data;
	remote_data = Remote_read_data();
	hexapod.mode_select(remote_data);
	hexapod.velocity_cal(remote_data);
	hexapod.body_angle_cal(remote_data);
	hexapod.body_position_cal(remote_data);
	hexapod.body_angle_and_pos_zero(remote_data);
}


/*
 * @brief 让机器人动起来
 * @param round_time 回合时间，单位ms
 */
void Hexapod::move(uint32_t round_time)
{
	// // 设置腿1-6的角度
	// for (int i = 0; i < 6; i++)
	// {
	// 	legs[i].set_thetas((gait_prg.actions[i].thetas[LegControl_round]) - leg_offset[i]); // 设置机械腿角度
	// 	legs[i].set_time(round_time);														// 设置机械腿移动时间
	// }
	// // 设置腿5的角度
	// if (gait_prg.actions[4].thetas[LegControl_round].angle[0] <= 0)
	// {
	// 	Thetas theta_temp;
	// 	theta_temp = (gait_prg.actions[4].thetas[LegControl_round]) - leg_offset[4];
	// 	theta_temp.angle[0] += 2 * PI;
	// 	legs[4].set_thetas(theta_temp); // 设置机械腿角度
	// }
	Thetas theta_temp;
	for (int i = 0; i < 6;i++)
	{
		theta_temp = (gait_prg.actions[i].thetas[LegControl_round]) - leg_offset[i];
		if(theta_temp.angle[0] <= -2.0f/3.0f*PI )
		{
			theta_temp.angle[0] += 2 * PI;
		}
		legs[i].set_thetas(theta_temp); // 设置机械腿角度
		legs[i].set_time(round_time);
		legs[i].move_DMA();
	}

	// legs[0].move_DMA();
	// legs[1].move_DMA();
	// legs[2].move_DMA();
	// legs[3].move_DMA();
	// legs[4].move_DMA();
	// legs[5].move_DMA();
}

