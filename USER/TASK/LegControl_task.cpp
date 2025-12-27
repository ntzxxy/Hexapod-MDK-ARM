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

#define SAFE_SPEED_X   100.0f   
#define SAFE_SPEED_Y   100.0f
#define SAFE_SPEED_W   0.80f    
#define RC_RANGE       800.0f   

using namespace std;

extern osMessageQId KeyQueueHandle;
ControlMode_t current_mode = MODE_IDLE; // 当前模式，默认空闲
UIMode_t current_ui_mode = UI_MODE_MOVEMENT;
RC_remote_data_t rc;
bool g_motor_power_on = true;
bool is_remote_active = false;		//遥控器是否使能

extern void Hexapod_Unload_All(void);
extern void Hexapod_Load_All(void);
extern void Hexapod_Read_And_Print_Pose(void);

uint8_t CALIB_LEG_INDEX = 3;//校准测试index，对哪条腿进行校准就修改相应的index

Thetas leg_cal_offsets[6] = {
    // Leg 0 (待校准，暂时全0)
    Thetas(0.0f, 0.0f * PI / 180.0f, -9.8f * PI / 180.0f), 
    
    // Leg 1 (你已经测好的数据)
		Thetas(0.0f, 0.0f * PI / 180.0f, 7.8f * PI / 180.0f), 
	
    
    // Leg 2-5 (先给0)
    Thetas(0.0f, -13.0f * PI / 180.0f, 4.8f * PI / 180.0f), 
    Thetas(12.852f * PI / 180.0f, -3.0f * PI / 180.0f, 5.8f * PI / 180.0f),
    Thetas(-21.85f * PI / 180.0f, 0.0f, 0.0f),
    Thetas(12.85f * PI / 180.0f, -3.0f * PI / 180.0f, 0.0f)
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
		
		for(int i=0; i<6; i++) 
		{
        hexapod.legs[i].set_cal_offset(leg_cal_offsets[i]);
    }
		DWT_Delay_Init();
		osDelay(300); // 启动延时，让系统稳定
		hexapod.starting_pose();
		
		static uint32_t code_time_start, code_time_end, code_time; // 用于计算程序运行时间，保证程序隔一段时间跑一遍
	      
    uint16_t move_time = 500; 
		
		uint32_t last_move_tick = 0;
		
		static uint32_t debug_print_tick = 0;

		while (1)
		{
						code_time_start = xTaskGetTickCount();
						key_deal();
						if(is_remote_active)
						{
							remote_deal();
						}

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
						float raw_vx = hexapod.velocity.Vx;
						float raw_vy = hexapod.velocity.Vy;
						bool reverse_round = false;
						switch (current_mode)
            {
                case MODE_IDLE:
                    // 空闲状态，什么都不做，或者发送软力矩指令
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
										if (raw_vy < 0) reverse_round = !reverse_round;
										// 如果 vx 为负（左移），反转一次
										if (raw_vx < 0) reverse_round = !reverse_round;
										gait_prg.set_velocity(hexapod.velocity);
										if (!reverse_round) {
												LegControl_round = (++LegControl_round) % N_POINTS;
										} else {
												if (LegControl_round == 0) LegControl_round = N_POINTS - 1;
												else LegControl_round--;
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

			osDelay(20); // 延时 20ms 等待舵机完成动作
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
					
						is_remote_active = false;

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
                        hexapod.velocity.Vx = 0.0f;
                        hexapod.velocity.Vy = -30.0f; // 这里的速度可以根据需要调整
                        hexapod.velocity.omega = 0.2f;
                        break;
                    
                    case 3: // 功能: 原地踏步 (测试步态)
                        current_mode = MODE_GAIT_RUN;
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
                    case 2: // 功能: 掉电 (开始掰腿)
                        // 安全检查：只有在示教模式下才允许掉电
                        if(current_mode == MODE_ACTION_TEACH) {
                            Hexapod_Unload_All(); 
                        }
                        break;
                    
                    case 3: // 功能: 读取并打印 (录制一帧)
                        if(current_mode == MODE_ACTION_TEACH) {
                            Hexapod_Read_And_Print_Pose();
                        }
                        break;
                    
                    case 4: // 功能: 上电 (锁定姿态/结束)
                        if(current_mode == MODE_ACTION_TEACH) {
                            Hexapod_Load_All();
                        }
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
	legs[2] = Leg(&huart2, 2, 1); // Leg 3: ID 2, Servo 1-3
  legs[3] = Leg(&huart2, 3, 4); // Leg 4: ID 3, Servo 4-6
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
    float std_femur = 40.0f * PI / 180.0f;   
    float std_tibia = -110.0f * PI / 180.0f; 

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
	if (this->mode == HEXAPOD_LOCK) 
    {
        // 锁定趴下
        this->body_pos.z = -90.0f;
        this->body_pos.x = 0;
        this->body_pos.y = 0;
    }
	else
	{
		if(this->mode == HEXAPOD_MOVE)
		{
		float height_offset = (float)remote_data.left_VETC / 800.0f * 50.0f; // +/- 50mm
    body_pos.z = -90.0f + height_offset;
		body_pos.x = 0;
    body_pos.y = 0;
		}
		else if (this->mode == HEXAPOD_BODY_ANGEL_CONTROL) // 若是身体位置控制模式则计算xy位置
		{
		body_pos.z = -90.0f;//暂时固定，后续修改  
		// body_pos.y += ROTATE_BODY_POS_SENSI * remote_data.right_VETC;
		// body_pos.x += ROTATE_BODY_POS_SENSI * remote_data.right_HRZC;
		body_pos.y = HEXAPOD_MAX_Y/800.0f * remote_data.right_VETC;
		body_pos.x = -HEXAPOD_MIN_X/800.0f * remote_data.right_HRZC;
		} 
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
		//body_angle.z = body_angle_fof[2].cal(0.001f * remote_data.right_HRZC);
		body_angle.z = 0;
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
   // 1. 读取数据
    rc = Remote_read_data();

    // 2. 简单的死区处理 (防止手抖)
    float dead_zone = 20.0f; 
    
    // 3. 计算输入速度 (直接映射)
    // 这里的 0.1f 和 -0.0005f 是你的速度系数，按需调整
    float input_Vy    = 0.0f; // 前后
    float input_Vx    = 0.0f; // 左右
		float input_Omega = 0.0f;
    if (rc.right_VETC > 220 || rc.right_VETC < -dead_zone) 
        input_Vy = (float)rc.right_VETC * 0.07f; 
        
    if (rc.right_HRZC > 220 || rc.right_HRZC < -dead_zone) 
        input_Vx = (float)rc.right_HRZC * 0.07f;
		
		if (rc.left_HRZC > dead_zone || rc.left_HRZC < -220) 
        input_Omega = (float)rc.left_HRZC * 0.07f;

		
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

        if (input_Vy != 0) 
        {
            // 前进或后退时，固定补偿 0.2f
            hexapod.velocity.omega = 0.2f; 
        }
        else if (input_Vx != 0) 
        {
            // 纯左右横移时，固定补偿 0.04f
            hexapod.velocity.omega = 0.04f;
        }
				else
				{
						hexapod.velocity.omega = -input_Omega;
				}
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
