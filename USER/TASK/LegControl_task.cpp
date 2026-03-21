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
#include "string.h"
#include "remote.h"
#include "auto_control.h"


#define SAFE_SPEED_X   100.0f   
#define SAFE_SPEED_Y   100.0f
#define SAFE_SPEED_W   0.80f    
#define RC_RANGE       800.0f   

using namespace std;

extern osMessageQId KeyQueueHandle;
void Hexapod_Show_Wave_Hand(void);
void Hexapod_Show_Circular_Gyrate(void);
void Action_Shake_Roll(void);
ControlMode_t current_mode = MODE_IDLE; // 当前模式，默认空闲
UIMode_t current_ui_mode = UI_MODE_MOVEMENT;
RC_remote_data_t rc;
bool g_motor_power_on = true;
bool is_remote_active = false;		//遥控器是否使能
bool is_MPU_ON = false;		//陀螺仪是否使能
bool is_uart_active = false;		//遥控器是否使能

uint32_t auto_mode_start_tick = 0; // 记录进入自动模式的起始时间
bool last_uart_active = false;    // 用于检测 is_uart_active 的上升沿

extern void Hexapod_Unload_All(void);
extern void Hexapod_Load_All(void);
extern void Hexapod_Read_And_Print_Pose(void);


uint8_t CALIB_LEG_INDEX = 3;//校准测试index，对哪条腿进行校准就修改相应的index

Thetas leg_cal_offsets[6] = {
    // Leg 0 (待校准，暂时全0)
    Thetas(0.0f, 0.0f * PI / 180.0f, -9.8f * PI / 180.0f), 
    
    // Leg 1 (你已经测好的数据)
		Thetas(6.5f * PI / 180.0f, 11.0f * PI / 180.0f, -7.8f * PI / 180.0f), 
	
    
    // Leg 2-5 (先给0)
    //Thetas(0.0f, -13.0f * PI / 180.0f, 0.8f * PI / 180.0f), 
    Thetas(7.852f * PI / 180.0f, -3.0f * PI / 180.0f, -4.8f * PI / 180.0f),
		Thetas(0.0f * PI / 180.0f, -13.0f * PI / 180.0f, 0.8f * PI / 180.0f),
    Thetas(-21.85f * PI / 180.0f, -5.0f * PI / 180.0f, -9.0f * PI / 180.0f),
    Thetas(12.85f * PI / 180.0f, -3.0f * PI / 180.0f, -12.0f * PI / 180.0f)
};


uint32_t LegControl_round; // 机器人回合数
Hexapod hexapod;		   // 机器人结构体
Gait_prg gait_prg;	  // 步态规划
uint32_t round_time;  // 回合时间
Velocity velocity;
uint8_t dma_buffer_uart1[60];
uint8_t dma_buffer_uart2[60];
uint8_t dma_buffer_uart3[60];
bool is_dirty = false;
bool is_wave = true;


Thetas leg_offset[6]; // 腿部关节角偏移，用于将舵机相对机器人本体的角度换算至相对舵机本身的角度
// 任务中需要使用的最小化 Leg 结构体实例 (仅代表第一条腿)
Leg my_leg;

// 定义腿末端目标位置（相对于腿起始端，单位mm）
Position3 target_pos; 

Position3 calib_target_pos(170.0f, 0.0f, -90.0f);

extern UART_HandleTypeDef huart2;
extern Thetas ikine(Position3 &pos);

void key_deal(void);
void remote_deal(void);
void auto_control(void);

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
		Remote_Init();
		AutoControl_Init(&huart5);
		
		for(int i=0; i<6; i++) 
		{
        hexapod.legs[i].set_cal_offset(leg_cal_offsets[i]);
    }
		DWT_Delay_Init();
		osDelay(300); // 启动延时，让系统稳定
		hexapod.starting_pose();
		
		static uint32_t code_time_start, code_time_end, code_time; // 用于计算程序运行时间，保证程序隔一段时间跑一遍
	      
    //uint16_t move_time = 500; 
		
		//uint32_t last_move_tick = 0;
		
		static uint32_t debug_print_tick = 0;

		while (1)
		{
						code_time_start = xTaskGetTickCount();
						key_deal();
			
						if (is_uart_active == true && last_uart_active == false) {
							// 说明刚才按了按键，刚进入 Auto 模式
							auto_mode_start_tick = xTaskGetTickCount(); // 记下现在的时刻
							}
							last_uart_active = is_uart_active; // 更新状态备份
						if(is_remote_active)
						{
							remote_deal();
						}
						
						if(is_uart_active)
						{
							if (xTaskGetTickCount() - auto_mode_start_tick < 3000) {
							velocity.Vx = 0;
							velocity.Vy = 0;
							velocity.omega = 0;
							gait_prg.set_velocity(velocity);
							}
							else
							{
								//auto_control();
								velocity.Vx = 0;
								velocity.Vy = 0; 
								velocity.omega = 0;
								gait_prg.set_velocity(velocity);
								
							}
								
						}

						hexapod.body_angle_cal(rc);		//机身被动式平衡
						if (xTaskGetTickCount() - debug_print_tick > 1000)
            {
                debug_print_tick = xTaskGetTickCount();
							/*
              APP_PRINT("RAW: CH1=%d,CH2=%d,CH3=%d,CH4=%d,SWA=%d,SWB=%d,SWC=%d,SWD=%d,VA=%d,VB=%d\r\n", 
              rc.right_HRZC, 
              rc.right_VETC, 
              rc.left_VETC,
							rc.left_HRZC,
							rc.func_sw,
							rc.mpu_sw,
							rc.gait_sw,
              rc.mode_sw,
							rc.knob_VRA,
							rc.knob_VRB);  
														
							
                // 1. 打印速度 (验证 0.125 系数是否生效，满舵应约等于 100.0)
							APP_PRINT("V:Vx=%.1f,Vy=%.1f,W=%.2f\r\n", 
                          hexapod.velocity.Vx, 
                          hexapod.velocity.Vy, 
                          hexapod.velocity.omega);
                
                // 2. 打印位置 (验证左摇杆平移和旋钮高度)
                // 左摇杆满舵 X/Y 应约等于 +/- 40.0
							APP_PRINT("Pos:X=%.1f,Y=%.1f,Z=%.1f\r\n", 
                          hexapod.body_pos.x, 
                          hexapod.body_pos.y, 
                          hexapod.body_pos.z);
                          
                // 3. 打印姿态 (验证右摇杆角度)
                // 右摇杆满舵应约等于 +/- 0.25 (14度)
							APP_PRINT("Ang:P=%.3f,R=%.3f,Y=%.3f\r\n", 
                          hexapod.body_angle.x, 
                          hexapod.body_angle.y, 
                          hexapod.body_angle.z);
                          
                APP_PRINT("Mode%d,MPU%d\r\n", hexapod.mode, hexapod.mpu_sw);
						*/		
            }
            // ==========================================================
						switch (current_mode)
            {
                case MODE_IDLE:
										if(current_ui_mode == UI_MODE_REMOTE)
										{
												//static float last_body_z = 0.0f;
												static Position3 last_angle(0,0,0);
												static Position3 last_body_pos(0,0,0);
        
												// 2. 计算当前目标高度（包含低通滤波）
												hexapod.body_position_cal(rc); 
												//float current_body_z = hexapod.body_pos.z;
											
												bool is_angle_changing = (abs(hexapod.body_angle.x - last_angle.x) > 0.002f || abs(hexapod.body_angle.y - last_angle.y) > 0.002f  || abs(hexapod.body_pos.y - last_body_pos.y) > 0.5f);


												// 3. 变化检测（设置 0.5mm 的死区，过滤传感器的细微噪声）
												if (abs(hexapod.body_pos.z - last_body_pos.z) > 0.5f || is_angle_changing)
												{
														// 更新旧值
														last_body_pos = hexapod.body_pos;
														last_angle = hexapod.body_angle;

														// 只有变化时才执行以下高能耗操作
														LegControl_round = 0; 
														gait_prg.gait_proggraming(); 

														// 执行移动（指令只在变化时下发一次）
														// 建议将 move 时间设短一点（如 30ms），增加实时跟随感
														hexapod.move(50);
												}
												osDelay(50);
										}
										
										if(current_ui_mode == UI_MODE_AUTO)
										{
												//static float last_body_z = 0.0f;
												static Position3 last_angle(0,0,0);
												static Position3 last_body_pos(0,0,0);
        
												// 2. 计算当前目标高度（包含低通滤波）
												hexapod.body_position_cal(rc); 
												//float current_body_z = hexapod.body_pos.z;
											
												bool is_angle_changing = (abs(hexapod.body_angle.x - last_angle.x) > 0.002f || abs(hexapod.body_angle.y - last_angle.y) > 0.002f  || abs(hexapod.body_pos.y - last_body_pos.y) > 0.5f);


												// 3. 变化检测（设置 0.5mm 的死区，过滤传感器的细微噪声）
												if (abs(hexapod.body_pos.z - last_body_pos.z) > 0.5f || is_angle_changing)
												{
														// 更新旧值
														last_body_pos = hexapod.body_pos;
														last_angle = hexapod.body_angle;

														// 只有变化时才执行以下高能耗操作
														LegControl_round = 0; 
														gait_prg.gait_proggraming(); 

														// 执行移动（指令只在变化时下发一次）
														// 建议将 move 时间设短一点（如 30ms），增加实时跟随感
														hexapod.move(50);
												}
												osDelay(50);
										}
										
                    break;

                case MODE_IK_TEST:
                    if (is_dirty) 
										{
												// 1. 运动学解算标准坐标
												Thetas solved = ikine(calib_target_pos);
												
												// 2. 将解算结果赋给当前正在校准的那条腿 (CALIB_LEG_INDEX)
												hexapod.legs[CALIB_LEG_INDEX].set_thetas(solved);
												
												// 3. 设定一个慢一点的时间 (1000ms)，防止动作太猛
												hexapod.legs[CALIB_LEG_INDEX].set_time(1000);
												
												// 4. 发送指令 (使用阻塞式 UART，确保发出去)
												hexapod.legs[CALIB_LEG_INDEX].move_UART();
												
												// 5. 清除标志位，任务完成，舵机保持在此位置不动
												is_dirty = false;
										}
										break;
									case MODE_GAIT_RUN:
												hexapod.body_position_cal(rc);
									
												LegControl_round = (++LegControl_round) % N_POINTS; // 控制回合自增长
								
										if(is_wave)
										{
											gait_prg.set_velocity(hexapod.velocity);
											gait_prg.run_wave_gait(); 
											round_time = gait_prg.get_pace_time() / N_POINTS;
										}
										else
										{
											gait_prg.set_velocity(hexapod.velocity);
											/*步态控制*/
											gait_prg.CEN_and_pace_cal();
											gait_prg.gait_proggraming();
											/*开始移动*/
											round_time = gait_prg.get_pace_time() / N_POINTS;
										}
										
										hexapod.move(round_time);
										// 计算程序运行时间
										code_time_end = xTaskGetTickCount();		 // 获取当前systick时间
										code_time = code_time_end - code_time_start; // 做差获取程序运行时间（8ms）
										if (code_time < round_time)
												osDelay(round_time - code_time); // 保证程序执行周期等于回合时间
										else
												osDelay(20); // 至少延时1ms
								continue;
										
								case MODE_BALANCE_TEST:
								{
										// [1] 锁定在站立相位
										LegControl_round = 0; 

										// [2] 强制重新计算逆运动学 (会将当前的 body_angle 应用到腿部坐标)
										gait_prg.gait_proggraming(); 

										// [3] 下发指令。move_time 设短一些（如 50-80ms），增加响应灵敏度
										hexapod.move(100); 

										// [4] 保证循环频率。自平衡需要较高的修正频率（建议 12.5Hz - 20Hz）
										osDelay(100);
								}
								continue;
                case MODE_RESET_ZERO:
                    // 归零逻辑...
                    break;
            }

			osDelay(10); // 延时 20ms 等待舵机完成动作
		}
	}
}

void key_deal(void)
{
    osEvent event = osMessageGet(KeyQueueHandle, 0); 
    if (event.status == osEventMessage)
    {
        uint8_t key_val = (uint8_t)event.value.v;

       if (key_val == 1) 
        {
            // 1. 切换 UI 模式 (循环)
            current_ui_mode = (UIMode_t)((current_ui_mode + 1) % UI_MODE_MAX);
						hexapod.body_pos.z = 0;
					
						is_remote_active = false;
						is_uart_active = false;

            // 2. 切换模式时的安全初始化逻辑
            switch (current_ui_mode)
            {
                case UI_MODE_MOVEMENT:
                    current_mode = MODE_IDLE; // 切回运动模式时，先站好待命
                    Hexapod_Load_All();       // 确保舵机有劲
                    break;
                
                case UI_MODE_TEACHING:
                    current_mode = MODE_ACTION_TEACH; // 切入示教状态机
                    // 此时不要自动掉电，防止刚切过来就趴下，交给 Key2 控制
                    break;
                
                case UI_MODE_DEBUG:
                    current_mode = MODE_IK_TEST; // 切入调试状态机
                    break;
                case UI_MODE_REMOTE:
										current_mode = MODE_IDLE; // 切回运动模式时，先站好待命
                    Hexapod_Load_All();       // 确保舵机有劲
										is_remote_active = true;
										break;
								case UI_MODE_BALANCE:
                    current_mode = MODE_IDLE; // 切回运动模式时，先站好待命
                    Hexapod_Load_All();       // 确保舵机有劲
                    break;
								case UI_MODE_AUTO:
										current_mode = MODE_IDLE; // 切回运动模式时，先站好待命
                    Hexapod_Load_All();       // 确保舵机有劲
										is_uart_active = true;
                default:
                    break;
            }
            return; // Key1 处理结束，直接返回
        }
        // === Key 2/3/4: 上下文功能键 ===
        switch (current_ui_mode)
        {
            // --- 场景 A: 运动控制界面 ---
            case UI_MODE_MOVEMENT:
                switch (key_val)
                {
                    case 2: // 功能: 启动行走 (前进)
                        current_mode = MODE_GAIT_RUN;
												is_wave = false;
                        hexapod.velocity.Vx = 0.0f;
                        hexapod.velocity.Vy = 30.0f; // 这里的速度可以根据需要调整
                        hexapod.velocity.omega = 2.0f;
                        break;
                    
                    case 3: // 功能: 原地踏步 (测试步态)
                        current_mode = MODE_GAIT_RUN;
												is_wave = false;
                        hexapod.velocity.Vx = -30.0f;
                        hexapod.velocity.Vy = 0.0f;
                        hexapod.velocity.omega = 0.04f;
                        break;
                    
                    case 4: // 功能: 停止/恢复站立
                        current_mode = MODE_IDLE;
                        hexapod.velocity.Vx = 0;
                        hexapod.velocity.Vy = 0;
                        break;
                }
                break;

            // --- 场景 B: 动作示教界面 ---
            case UI_MODE_TEACHING:
                switch (key_val)
                {
                   case 2: // 功能: 启动行走 (前进)
                        current_mode = MODE_GAIT_RUN;
												is_wave = true;
                        hexapod.velocity.Vx = 0.0f;
                        hexapod.velocity.Vy = 30.0f; // 这里的速度可以根据需要调整
                        hexapod.velocity.omega = 0.2f;
                        break;
                    
                    case 3: // 功能: 原地踏步 (测试步态)
                        current_mode = MODE_GAIT_RUN;
												is_wave = true;
                        hexapod.velocity.Vx = -30.0f;
                        hexapod.velocity.Vy = 0.0f;
                        hexapod.velocity.omega = 0.04f;
                        break;
                    
                    case 4: // 功能: 停止/恢复站立
                        current_mode = MODE_IDLE;
                        hexapod.velocity.Vx = 0;
                        hexapod.velocity.Vy = 0;
                        break;

                        break;
                }
                break;

            // --- 场景 C: IK/单腿调试界面 (可选) ---
            case UI_MODE_DEBUG:
                 switch (key_val)
                {
                    case 2: 
                    current_mode = MODE_IK_TEST; // 触发动作
                    is_dirty = true;
                    break;

										case 3: // Z轴 + (抬高腿)
												calib_target_pos.z += 5.0f;
												current_mode = MODE_IK_TEST; 
												is_dirty = true;             
												break;

										case 4: // Z轴 - (放低腿)
												calib_target_pos.z -= 5.0f;
												current_mode = MODE_IK_TEST; 
												is_dirty = true;
												break;
                }
                break;
								
						case UI_MODE_BALANCE:
                 switch (key_val)
                {
                    case 2: 
                    current_mode = MODE_BALANCE_TEST; // 触发动作
										is_MPU_ON = true;
                    break;

										case 3: // Z轴 + (抬高腿)
												break;

										case 4: // 功能: 停止/恢复站立
                        current_mode = MODE_IDLE;
												is_MPU_ON = false;
                        break;
                }
                break;
								
						case UI_MODE_SHOW: // 假设在遥控/演示模式下
								switch (key_val)
								{
										case 2: // 按键 2 启动演示
												Hexapod_Show_Wave_Hand();
												break;
										case 3: // 按下 3 号键开启“圆周摆动”
												Hexapod_Show_Circular_Gyrate();
												break;
										case 4:
												Action_Shake_Roll();
												osDelay(100); // 动作切换缓冲
												Position3 reset_pos(0,0,0);
												gait_prg.set_body_rotate_angle(reset_pos);
												hexapod.move(1000);
												break;
								}
								break;
								
					
            default:
                break;
        }
    }
}


// 初始化腿部变量并初始化串口，使能串口发送
void Hexapod::Init(void)
{
	legs[0] = Leg(&huart1, 0, 4); // Leg 1: ID 0, Servo 1-3
  legs[1] = Leg(&huart1, 1, 1); // Leg 2: ID 1, Servo 4-6
	legs[2] = Leg(&huart2, 2, 4); // Leg 3: ID 2, Servo 1-3
  legs[3] = Leg(&huart2, 3, 1); // Leg 4: ID 3, Servo 4-6
	legs[4] = Leg(&huart3, 4, 1); // Leg 5: ID 4, Servo 1-3
  legs[5] = Leg(&huart3, 5, 4); // Leg 6: ID 5, Servo 4-6
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

void Hexapod::starting_pose()
{   
    // 1. 设定慢速 (2秒)
    for(int i=0; i<6; i++) {
        legs[i].set_time(2000);
    }
    
    // 2. 发送站立角度 (注意：这里用的是你的宏定义)
    float std_femur = 50.0f * PI / 180.0f;   
    float std_tibia = -120.0f * PI / 180.0f; 

    for(int i=0; i<6; i++) {
        // 计算目标角度：标准站立姿态 - 修正后的偏移量
        // 这样可以确保无论你刚才怎么改 offset，这里都是让它去物理上的“站立位”
        Thetas stand_target(0.0f , std_femur, std_tibia);
        legs[i].set_thetas(stand_target);
        legs[i].move_UART();
        osDelay(10); // 稍微错开，防止电源冲击
    }
    
    // 3. 必须等待动作完成
    osDelay(2500); 
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
		velocity.Vx = velocity_fof[0].cal(0.1f * remote_data.right_HRZC);
		velocity.Vy = velocity_fof[1].cal(0.1f * remote_data.right_VETC);
		velocity.omega = velocity_fof[2].cal(-0.0005f * remote_data.left_HRZC);
	}
	if(velocity.Vx>-0.1f && velocity.Vx<0.1f)velocity.Vx=0;
	if(velocity.Vy>-0.1f && velocity.Vy<0.1f)velocity.Vy=0;
	if(velocity.omega>-0.1f && velocity.omega<0.1f)velocity.omega=0;
	gait_prg.set_velocity(velocity);
}

void Hexapod::body_position_cal(const RC_remote_data_t &remote_data)
{
		if (current_ui_mode == UI_MODE_REMOTE) 
    {
        // HT-10A 原始数据范围约 192~1792，减去 992 得到 -800~800 [cite: 23, 27]
        float raw_offset = (float)remote_data.knob_VRA; // remote.c 已处理过减法 
        
        // 映射到 +/- 25mm。注意：2.4G 遥控器上电瞬间如果没信号，
        // 你的 remote.c 会返回 zero_data，此时 raw_offset 为 0，高度保持 -74mm 不会乱动 
        float z_offset = raw_offset / 800.0f * 75.0f;
			
				this->body_pos.y = (float)remote_data.knob_VRB / 800.0f * 40.0f;
				value_limit(this->body_pos.y, HEXAPOD_MIN_Y, HEXAPOD_MAX_Y); // 加上限幅
        
        // 安全红线限制
        value_limit(z_offset, -35.0f, 75.0f);
        this->body_pos.z = z_offset;
    }
		else if (current_ui_mode == UI_MODE_AUTO) 
    {
        // HT-10A 原始数据范围约 192~1792，减去 992 得到 -800~800 [cite: 23, 27]
        float raw_offset = (float)auto_data.body_h; // remote.c 已处理过减法 
        
        // 映射到 +/- 25mm。注意：2.4G 遥控器上电瞬间如果没信号，
        // 你的 remote.c 会返回 zero_data，此时 raw_offset 为 0，高度保持 -74mm 不会乱动 

        float z_offset = raw_offset;
    
        // 安全红线限制
        value_limit(z_offset, -35.0f, 75.0f);
        this->body_pos.z = z_offset;
    }
    else 
    {
        // 非遥控模式下，高度偏移回归 0
        this->body_pos.z = 0;
    }

    // [5] 低通滤波：确保升降过程丝滑，保护舵机
    this->body_pos.z = body_pos_fof[0].cal(this->body_pos.z);
		this->body_pos.y = body_pos_fof[1].cal(this->body_pos.y);

    // [6] 同步更新：让步态算法感知新的高度基准
    gait_prg.set_body_position(this->body_pos);
}

void Hexapod::body_angle_cal(const RC_remote_data_t &remote_data)
{
    // 1. 获取当前欧拉角（转换为弧度）
    float cur_x_rad = mpu6050.angle.x * (PI / 180.0f); // Pitch
    float cur_y_rad = mpu6050.angle.y * (PI / 180.0f); // Roll
    
    // 2. 获取目标角度（弧度）
    float set_x_rad = mpu_angle_set.x * (PI / 180.0f);
    float set_y_rad = mpu_angle_set.y * (PI / 180.0f);

    if (is_MPU_ON) 
    {
        // --- 新增：死区处理逻辑 ---
        // 定义死区角度（例如：0.5度），将其转为弧度
        const float dead_zone_rad = 0.5f * (PI / 180.0f); 
        
        // 计算当前偏差
        float diff_x = set_x_rad - cur_x_rad;
        float diff_y = set_y_rad - cur_y_rad;

        // 如果偏差在死区范围内，强制让当前值等于目标值，使 PID 输出为 0
        float input_cur_x = cur_x_rad;
        float input_cur_y = cur_y_rad;

        if (abs(diff_x) < dead_zone_rad) input_cur_x = set_x_rad; 
        if (abs(diff_y) < dead_zone_rad) input_cur_y = set_y_rad;
        // -----------------------

        // 3. 计算 PID
        // 传入经过死区处理后的“当前值”
        body_angle.x = this->mpu_pid_x.cal(input_cur_x, set_x_rad); 
        body_angle.y = -this->mpu_pid_y.cal(input_cur_y, set_y_rad); 
    }
    else 
    {
        this->mpu_flag = false;
        body_angle.x = 0;
        body_angle.y = 0;
        body_angle.z = 0;
    }

    // 4. 限幅
    value_limit(body_angle.x, HEXAPOD_MIN_X_ROTATE, HEXAPOD_MAX_X_ROTATE);
    value_limit(body_angle.y, HEXAPOD_MIN_Y_ROTATE, HEXAPOD_MAX_Y_ROTATE);

    // 5. 同步给步态算法
    gait_prg.set_body_rotate_angle(body_angle);
}

/*
 *@brief 检查拨轮数据，符合要求就归零
 *@param remote_data 遥控数据
 */
void Hexapod::body_angle_and_pos_zero(const RC_remote_data_t &remote_data)
{
	if (remote_data.func_sw == 3)
	{
		this->body_angle.zero();
		this->body_pos.x = 0.0f;
    this->body_pos.y = 0.0f;
    this->body_pos.z = -90.0f;
		this->mpu_angle_set.zero();
	}
}

void Hexapod::mode_select(const RC_remote_data_t &remote_data)
{
	switch (remote_data.mode_sw)
	{
	case 1:					 // 拨杆在上面
		this->mode = HEXAPOD_LOCK;
    current_mode = MODE_IDLE;
		break;
	case 2:								   // 拨杆在中间
		this->mode = HEXAPOD_BODY_ANGEL_CONTROL; // 借用这个枚举名，实际功能增强了
    current_mode = MODE_GAIT_RUN; // 引擎启动
		break;
	case 3:								 // 拨杆在下面
		this->mode = HEXAPOD_MOVE;
    current_mode = MODE_GAIT_RUN; // 引擎启动
	default:
		break;
	}
	switch (remote_data.mpu_sw)
	{
	case 0:
		mpu_sw = MPU_ON;		//下面
		break;
	case 1:
		mpu_sw = MPU_OFF;		//上面
		break;
	default:
		break;
	}
}

void remote_deal(void)
{
    rc = Remote_read_data();
		static int16_t last_mode_sw = 0;
	
		if (last_mode_sw == 2 && rc.mode_sw != 2)
    {
        // 1. 强制清零目标角度
        hexapod.body_angle.x = 0.0f;
        hexapod.body_angle.y = 0.0f;
        gait_prg.set_body_rotate_angle(hexapod.body_angle);
        
        LegControl_round = 0; 
        gait_prg.gait_proggraming(); 

        for(int i=0; i<6; i++) {
            hexapod.legs[i].set_time(1000); 
        }

        hexapod.move(1000); 

        osDelay(1000); 

        hexapod.body_angle_fof[0].cal(0.0f);
        hexapod.body_angle_fof[1].cal(0.0f);
    }
    last_mode_sw = rc.mode_sw; // 更新模式记录

    // 2. 简单的死区处理 (防止手抖)
    float dead_zone = 220.0f; 
    if(rc.mode_sw == 3)
		{
			is_wave = false;
		}
		else if(rc.mode_sw == 1)
		{
			is_wave = true;
		}
		else if(rc.mode_sw == 2)
		{
			is_wave = false;
			if ( rc.left_VETC > dead_zone || rc.left_VETC < -dead_zone)
            hexapod.body_angle.x += (float)rc.left_VETC * ROTATE_BODY_ANGLE_SENSI; // 累加式
        
        if ( rc.left_HRZC > dead_zone || rc.left_HRZC < -dead_zone)
            hexapod.body_angle.y -= (float)rc.left_HRZC * ROTATE_BODY_ANGLE_SENSI;

        // 严格限幅保护
        value_limit(hexapod.body_angle.x, HEXAPOD_MIN_X_ROTATE, HEXAPOD_MAX_X_ROTATE);
        value_limit(hexapod.body_angle.y, HEXAPOD_MIN_Y_ROTATE, HEXAPOD_MAX_Y_ROTATE);
        
        // 更新到算法中
        gait_prg.set_body_rotate_angle(hexapod.body_angle);
		}
		
		if(rc.mpu_sw == 0)
		{
			is_MPU_ON = false;
		}
		else if(rc.mpu_sw == 1)
		{
			is_MPU_ON = true;
		}
		if(rc.gait_sw == 0)
		{
			gait_prg.is_obstacle_mode = false;
		}
		else if(rc.gait_sw == 1)
		{
			gait_prg.is_obstacle_mode = true;
		}
		
    // 3. 计算输入速度 (直接映射)
    float input_Vy    = 0.0f; // 前后
    float input_Vx    = 0.0f; // 左右
		float input_Omega = 0.0f;
		if(rc.mode_sw != 2)
		{
			if (rc.right_VETC > dead_zone || rc.right_VETC < -dead_zone) 
        input_Vy = (float)rc.right_VETC * 0.05f; 
        
			if (rc.right_HRZC > dead_zone || rc.right_HRZC < -dead_zone) 
					input_Vx = (float)rc.right_HRZC * 0.05f;
			
			if (rc.left_HRZC > dead_zone || rc.left_HRZC < -dead_zone) 
					input_Omega = (float)rc.left_HRZC * 0.05f;
			
			 hexapod.body_angle.x = hexapod.body_angle_fof[0].cal(0.0f);
       hexapod.body_angle.y = hexapod.body_angle_fof[1].cal(0.0f);
		}
    else if(rc.mode_sw == 2)
		{
			if (rc.right_VETC > dead_zone || rc.right_VETC < -dead_zone) 
        input_Vy = (float)rc.right_VETC * 0.05f; 
			
			if (rc.right_HRZC > dead_zone || rc.right_HRZC < -dead_zone) 
				input_Omega = (float)rc.right_HRZC * 0.05f;
			
		}
		
    // 4. 核心逻辑：完全模仿按键
    if (input_Vx == 0 && input_Vy == 0 && input_Omega == 0)
    {
        // === 没推摇杆 = 按下 Key 4 (停止) ===
        current_mode = MODE_IDLE;
        
        hexapod.velocity.Vx = 0;
        hexapod.velocity.Vy = 0;
        hexapod.velocity.omega = 0;
    }
    else
    {
        // === 推了摇杆 = 按下 Key 2 (行走) ===
        current_mode = MODE_GAIT_RUN;

        // 赋值速度
        hexapod.velocity.Vx = input_Vx;
        hexapod.velocity.Vy = input_Vy;

       
				hexapod.velocity.omega = -input_Omega;
				
        // 为了防止万一Z是0导致趴下，加这一句最保险
        // 如果你确信按键模式下Z没问题，这行删了也行，但留着绝对没坏处
        //hexapod.body_pos.z = -90.0f; 
    }
}

void auto_control(void)
{
		static int16_t last_mode_sw = 0;
	
		if (last_mode_sw == 2 && auto_data.mode_sw != 2)
    {
        // 1. 强制清零目标角度
        hexapod.body_angle.x = 0.0f;
        hexapod.body_angle.y = 0.0f;
        gait_prg.set_body_rotate_angle(hexapod.body_angle);
        
        LegControl_round = 0; 
        gait_prg.gait_proggraming(); 

        for(int i=0; i<6; i++) {
            hexapod.legs[i].set_time(1000); 
        }

        hexapod.move(1000); 

        osDelay(1000); 

        hexapod.body_angle_fof[0].cal(0.0f);
        hexapod.body_angle_fof[1].cal(0.0f);
    }
    last_mode_sw = auto_data.mode_sw; // 更新模式记录

    // 2. 简单的越线处理 (防止超速)
    float safe_zone = 40.0f; 
    if(auto_data.mode_sw == 3)
		{
			is_wave = false;
		}
		else if(auto_data.mode_sw == 1)
		{
			is_wave = true;
		}
		
		if(auto_data.gait_sw == 0)
		{
			gait_prg.is_obstacle_mode = false;
		}
		else if(auto_data.gait_sw == 1)
		{
			gait_prg.is_obstacle_mode = true;
		}
		
    // 3. 计算输入速度 (直接映射)
    float input_Vy    = 0.0f; // 前后
    float input_Vx    = 0.0f; // 左右
		float input_Omega = 0.0f;
		if(auto_data.mode_sw != 2)
		{
			if (auto_data.vy_raw < safe_zone || auto_data.vy_raw > -safe_zone) 
        input_Vy = (float)auto_data.vy_raw; 
      else if (auto_data.vy_raw > safe_zone || auto_data.vy_raw < -safe_zone) 
				input_Vy = 0;
			
			if (auto_data.vx_raw < safe_zone || auto_data.vx_raw > -safe_zone) 
					input_Vx = (float)auto_data.vx_raw;
			else if (auto_data.vx_raw > safe_zone || auto_data.vx_raw < -safe_zone)  
					input_Vx = 0;
			
			if (auto_data.w_raw < safe_zone || auto_data.w_raw > -safe_zone) 
					input_Omega = (float)auto_data.w_raw;
			else if (auto_data.w_raw > safe_zone || auto_data.w_raw < -safe_zone)  
					input_Omega = 0;
			
			 hexapod.body_angle.x = hexapod.body_angle_fof[0].cal(0.0f);
       hexapod.body_angle.y = hexapod.body_angle_fof[1].cal(0.0f);
		}
   
		
    // 4. 核心逻辑：完全模仿按键
    if (input_Vx == 0 && input_Vy == 0 && input_Omega == 0)
    {
        // === 没推摇杆 = 按下 Key 4 (停止) ===
        current_mode = MODE_IDLE;
        
        hexapod.velocity.Vx = 0;
        hexapod.velocity.Vy = 0;
        hexapod.velocity.omega = 0;
    }
    else
    {
        // === 推了摇杆 = 按下 Key 2 (行走) ===
        current_mode = MODE_GAIT_RUN;

        // 赋值速度
        hexapod.velocity.Vx = input_Vx;
        hexapod.velocity.Vy = input_Vy;

       
				hexapod.velocity.omega = -input_Omega;
				
        // 为了防止万一Z是0导致趴下，加这一句最保险
        // 如果你确信按键模式下Z没问题，这行删了也行，但留着绝对没坏处
        //hexapod.body_pos.z = -90.0f; 
    }
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
    
    // --- 第一步：纯计算 (CPU 只要算好数据，存进各自的 buffer) ---
    for (int i = 0; i < 6; i++)
    {
        // 1. 计算角度
        theta_temp = (gait_prg.actions[i].thetas[LegControl_round]) - leg_offset[i];
        if(theta_temp.angle[0] <= -2.0f/3.0f*PI ) 
					theta_temp.angle[0] += 2 * PI;
        
        // 2. 更新对象状态
        legs[i].set_thetas(theta_temp);
        legs[i].set_time(round_time);
        
        // 3. 【关键】只准备数据，不发送！
        legs[i].prepare_move_buffer(); 
    }
		// UART1 组: Leg 0 + Leg 1
    memcpy(dma_buffer_uart1, legs[0].get_send_buffer(), 30);      // 拷入Leg 0
    memcpy(dma_buffer_uart1 + 30, legs[1].get_send_buffer(), 30); // 接上Leg 1
    
    // UART2 组: Leg 2 + Leg 3
    memcpy(dma_buffer_uart2, legs[2].get_send_buffer(), 30);
    memcpy(dma_buffer_uart2 + 30, legs[3].get_send_buffer(), 30);
    
    // UART3 组: Leg 4 + Leg 5
    memcpy(dma_buffer_uart3, legs[4].get_send_buffer(), 30);
    memcpy(dma_buffer_uart3 + 30, legs[5].get_send_buffer(), 30);

    // --- 第三步：开启 DMA 并行发送 (瞬间完成) ---
    
    // 1. 打开所有腿的发送门 (74HCT126 的 OE 引脚)
    // 注意：因为 Leg0 和 Leg1 共用串口，我们必须让它俩的 TXE 都打开，信号才能传给两个插座
    for(int i=0; i<6; i++) {
        legs[i].TX_Enable(); 
    }
		HAL_UART_Transmit_DMA(&huart1, dma_buffer_uart1, 60);
    HAL_UART_Transmit_DMA(&huart2, dma_buffer_uart2, 60);
    HAL_UART_Transmit_DMA(&huart3, dma_buffer_uart3, 60);

	// legs[0].move_DMA();
	// legs[1].move_DMA();
	// legs[2].move_DMA();
	// legs[3].move_DMA();
	// legs[4].move_DMA();
	// legs[5].move_DMA();
}

void Hexapod_Unload_All(void)
{
    // 遍历6条腿
    for(int i = 0; i < 6; i++)
    {
        hexapod.legs[i].unload(); 
        // 适当延时，防止指令太快总线吞吐不过来
        osDelay(2); 
    }
		g_motor_power_on = false;
    APP_PRINT("All Servos Unloaded!\r\n");
}

void Hexapod_Load_All(void)
{
    for(int i = 0; i < 6; i++)
    {
        hexapod.legs[i].load();
        osDelay(2);
    }
		g_motor_power_on = true;
    APP_PRINT("All Servos Loaded!\r\n");
}

void Hexapod_Read_And_Print_Pose(void)
{
    APP_PRINT("Current Pose Data \r\n");
   
    for(int i = 0; i < 6; i++)
    {
        // 1. 读取单条腿的3个关节
        // 注意：Leg::read_angle 内部是 HAL_UART_Receive 阻塞等待，这很安全
        hexapod.legs[i].read_angle(1); // Coxa
        hexapod.legs[i].read_angle(2); // Femur
        hexapod.legs[i].read_angle(3); // Tibia

        // 2. 获取读取后更新到对象中的角度
        Thetas t = hexapod.legs[i].get_current_thetas();

        // 3. 打印格式化数据 (弧度值)，方便直接复制到代码
        // 例如: {0.52, 0.33, -1.20}, // Leg 0
        APP_PRINT("  {%.4ff, %.4ff, %.4ff}, // Leg %d\r\n", t.angle[0], t.angle[1], t.angle[2], i);
        
        // 4. 这里的延时很重要！
        // 因为你的Leg0和Leg1共用串口，必须等前一个读完，总线空闲了再读下一个
        osDelay(5); 
    }
    
}

void Hexapod_Show_Wave_Hand(void)
{
    Thetas theta_cmd;
    
    // --- 步骤 1: 抬起手臂 (Leg 0) ---
    // 目标：Coxa=0, Femur=75°, Tibia=-90°
    theta_cmd.angle[0] = 0.0f;
    theta_cmd.angle[1] = 75.0f * PI / 180.0f;
    theta_cmd.angle[2] = -70.0f * PI / 180.0f;
    
    hexapod.legs[0].set_thetas(theta_cmd);
    hexapod.legs[0].set_time(1000); // 1秒平滑抬起
    hexapod.legs[0].move_UART();
    osDelay(1200); // 等待动作到位
    for(int i = 0; i < 3; i++)
    {
        theta_cmd.angle[0] = 13.0f * PI / 180.0f;
        hexapod.legs[0].set_thetas(theta_cmd);
        hexapod.legs[0].set_time(800); // 往一个方向走 0.8s
        hexapod.legs[0].move_UART();
        osDelay(900);
        
        theta_cmd.angle[0] = -13.0f * PI / 180.0f;
        hexapod.legs[0].set_thetas(theta_cmd);
        hexapod.legs[0].set_time(800);
        hexapod.legs[0].move_UART();
        osDelay(900);
    }
    
    // 回正第一关节
    theta_cmd.angle[0] = 0.0f;
    hexapod.legs[0].set_thetas(theta_cmd);
    hexapod.legs[0].set_time(400);
    hexapod.legs[0].move_UART();
    osDelay(500);

    for(int j = 0; j < 2; j++)
    {
        // 勾向 -110°
        theta_cmd.angle[2] = -90.0f * PI / 180.0f;
        hexapod.legs[0].set_thetas(theta_cmd);
        hexapod.legs[0].set_time(600); // 勾一下 0.6s
        hexapod.legs[0].move_UART();
        osDelay(700);
        
        // 回到 -90°
        theta_cmd.angle[2] = -70.0f * PI / 180.0f;
        hexapod.legs[0].set_thetas(theta_cmd);
        hexapod.legs[0].set_time(600);
        hexapod.legs[0].move_UART();
        osDelay(700);
    }

    hexapod.starting_pose(); 
}

void Hexapod_Show_Circular_Gyrate(void)
{
    Position3 dance_angle(0.0f, 0.0f, 0.0f); 
    float dance_time = 0.0f;
    const float dt = 0.025f;      // 25ms 刷新频率
    const float period = 4.0f;     // 4秒转一圈
    const float amplitude = 0.10f; // 摆动幅度约 6 度

    // 运行两周：2周 * 4秒 / 0.025s = 320次循环
    for(uint32_t i = 0; i < 320; i++) 
    {
        dance_time += dt;
        float omega = 2.0f * PI / period;

        dance_angle.y = amplitude * sin(omega * dance_time); // Roll
        dance_angle.x = amplitude * cos(omega * dance_time); // Pitch

        gait_prg.set_body_rotate_angle(dance_angle); 
        
        LegControl_round = 0; 
        gait_prg.gait_proggraming(); 

        hexapod.move(30); 

        osDelay(25);
    }

    // 动作复位
    dance_angle.x = 0; 
		dance_angle.y = 0; 
		dance_angle.z = 0;
    gait_prg.set_body_rotate_angle(dance_angle);
    hexapod.move(1000); 
    osDelay(1000);
}

void Action_Shake_Roll(void) {
    Position3 shake_angle; // 定义一个位置/角度结构体
    const float amp = 5.0f * PI / 180.0f; 
    
    for(int i = 0; i < 40; i++) {
        // 计算 Roll 角度
        float angle_y = amp * sin(2.0f * 2.0f * PI * (i * 0.025f)); 
        
        // 显式给成员赋值，避免构造函数报错
        shake_angle.x = 0.0f;
        shake_angle.y = angle_y;
        shake_angle.z = 0.0f;
        
        gait_prg.set_body_rotate_angle(shake_angle); // 下发角度
        LegControl_round = 0; // 锁定支撑相
        gait_prg.gait_proggraming(); // 执行逆运动学
        hexapod.move(25); // 移动指令
        osDelay(25);
    }
}

void Action_Shake_Pitch(void) {
    Position3 shake_angle; 
    float current_time = 0.0f;
    const float dt = 0.025f;       // 保持 25ms 一帧 (40Hz)
    const float amplitude = 0.087f; // 幅度：约 5 度 (弧度制)
    
    // --- 修改速度看这里 ---
    const float freq = 2.0f;       // 2.0 代表 1 秒 2 个来回。改成 1.0 则变慢。
    const float duration = 1.0f;    // 持续时间：1 秒
    int total_steps = (int)(duration / dt); // 计算总步数 (40步)

    for(int i = 0; i < total_steps; i++) {
        current_time = i * dt;
        
        // 计算当前时刻的 Pitch 角度
        float angle_x = amplitude * sin(2.0f * PI * freq * current_time);
        
        // 显式赋值，防止 Position3 构造函数报错
        shake_angle.x = angle_x;
        shake_angle.y = 0.0f;
        shake_angle.z = 0.0f;
        
        // 1. 设置角度
        gait_prg.set_body_rotate_angle(shake_angle);
        // 2. 锁定步态相位
        LegControl_round = 0;
        // 3. 逆运动学计算
        gait_prg.gait_proggraming();
        // 4. 下发指令 (注意：move 的参数最好等于或略大于 dt，保证平滑)
        hexapod.move(25);
        
        osDelay(25); 
    }

    // 动作收尾，缓慢归零
    shake_angle.x = 0;
    gait_prg.set_body_rotate_angle(shake_angle);
    hexapod.move(200);
}
