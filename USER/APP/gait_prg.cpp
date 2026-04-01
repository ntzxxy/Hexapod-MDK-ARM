#include "gait_prg.h"
#include "cmsis_os.h"
#include <cmath>
#include "remote.h"
#include "my_math.h"
#include "string.h"
#include "LegControl_task.h"
//#include "arm_math.h"
using namespace std;

// 全局变量
extern uint32_t LegControl_round; // 控制回合
bool is_obstacle_mode = true;
volatile bool is_gate_test = true;
// 函数
static Position3 fkine(Thetas thetas);
Thetas ikine(Position3 &pos);

void Gait_prg::Init()
{
    // 计算机械腿相对于起始端的末端坐标
    Pws[0] = fkine(Thetas(PI / 4, THETA_STAND_2, THETA_STAND_3));
    Pws[1] = fkine(Thetas(0, THETA_STAND_2, THETA_STAND_3));
    Pws[2] = fkine(Thetas(-PI / 4, THETA_STAND_2, THETA_STAND_3));
    Pws[3] = fkine(Thetas(3 * PI / 4, THETA_STAND_2, THETA_STAND_3));
    Pws[4] = fkine(Thetas(PI, THETA_STAND_2, THETA_STAND_3));
    Pws[5] = fkine(Thetas(5 * PI / 4, THETA_STAND_2, THETA_STAND_3));
    // 默认站立坐标，这里copy一份
    memcpy(Pws_default, Pws, sizeof(Position3) * 6);
    // 计算各个机械腿起始端相对于机器人中心的坐标
    P_legs[0] = Position3(CHASSIS_FRONT_WIDTH / 2, CHASSIS_LEN / 2, 0);
    P_legs[1] = Position3(CHASSIS_WIDTH / 2, 0, 0);
    P_legs[2] = Position3(CHASSIS_FRONT_WIDTH / 2, -CHASSIS_LEN / 2, 0);
    P_legs[3] = Position3(-CHASSIS_FRONT_WIDTH / 2, CHASSIS_LEN / 2, 0);
    P_legs[4] = Position3(-CHASSIS_WIDTH / 2, 0, 0);
    P_legs[5] = Position3(-CHASSIS_FRONT_WIDTH / 2, -CHASSIS_LEN / 2, 0);
}

/*
 * 正运动解算
 */
static Position3 fkine(Thetas thetas)
{
    Position3 position3(cos(thetas.angle[0]) * (LEG_LEN1 + LEG_LEN3 * cos(thetas.angle[1] + thetas.angle[2]) + LEG_LEN2 * cos(thetas.angle[1])),
                        sin(thetas.angle[0]) * (LEG_LEN1 + LEG_LEN3 * cos(thetas.angle[1] + thetas.angle[2]) + LEG_LEN2 * cos(thetas.angle[1])),
                        LEG_LEN3 * sin(thetas.angle[1] + thetas.angle[2]) + LEG_LEN2 * sin(thetas.angle[1]));

    return position3;
}

/*
 * 逆运动解算
 */
Thetas ikine(Position3 &pos)
{
    static Position3 pos1;
    static float R, Lr, alpha_r, alpha1, alpha2;
    pos1 = pos;
    R = sqrt(pow(pos1.x, 2) + pow(pos1.y, 2));
    Lr = sqrt(pow(pos1.z, 2) + pow((R - LEG_LEN1), 2));
    alpha_r = atan(-pos1.z / (R - LEG_LEN1));
    alpha1 = acos((pow(LEG_LEN2, 2) + pow(Lr, 2) - pow(LEG_LEN3, 2)) / (2 * Lr * LEG_LEN2));
    alpha2 = acos((pow(Lr, 2) + pow(LEG_LEN3, 2) - pow(LEG_LEN2, 2)) / (2 * Lr * LEG_LEN3));
    Thetas thetas(atan2(pos1.y, pos1.x), alpha1 - alpha_r, -(alpha1 + alpha2));
    value_limit(thetas.angle[1], MIN_JOINT2_RAD, MAX_JOINT2_RAD);
    value_limit(thetas.angle[2], MIN_JOINT3_RAD, MAX_JOINT3_RAD);
    return thetas;
}

float Gait_prg::move_point()
{
    float fun,m_velocity;
    m_velocity = sqrt(pow(velocity.Vx,2)+pow(velocity.Vy,2));
    fun = (body_pos.x * velocity.Vx + body_pos.y * velocity.Vy)/(m_velocity)*K_W;
    return fun;
}

void Gait_prg::set_body_rotate_angle(Position3 &rotate_angle)
{
    this->rotate_angle = rotate_angle;
}

/*
 *@brief 通过机身旋转角度，计算机械腿末端位置
 *@param point 腿末端相对于起始端的坐标，
 *@param index 腿的编号
 */
Position3 Gait_prg::hexapod_rotate(Position3 &point, uint32_t index)
{
    Position3 retvel;
    retvel.x = cos(rotate_angle.y) * cos(rotate_angle.z) * (P_legs[index].x + point.x) - cos(rotate_angle.y) * sin(rotate_angle.z) * (P_legs[index].y + point.y);
    retvel.y = (cos(rotate_angle.x) * sin(rotate_angle.z) + cos(rotate_angle.z) * sin(rotate_angle.x) * sin(rotate_angle.y)) * (P_legs[index].x + point.x) + (cos(rotate_angle.x) * cos(rotate_angle.z) - sin(rotate_angle.x) * sin(rotate_angle.y) * sin(rotate_angle.z)) * (P_legs[index].y + point.y);
    retvel.z = point.z + (sin(rotate_angle.x) * sin(rotate_angle.z) - cos(rotate_angle.x) * cos(rotate_angle.z) * sin(rotate_angle.y)) * (P_legs[index].x + point.x) + (cos(rotate_angle.z) * sin(rotate_angle.x) + cos(rotate_angle.x) * sin(rotate_angle.y) * sin(rotate_angle.z)) * (P_legs[index].y + point.y);
    retvel = retvel - P_legs[index];
    return retvel;
}

/*
 *@brief 设置机器人高度
 *@param height 机器人的高度
 */
void Gait_prg::set_height(float height)
{
    for (int i = 0; i < 6; i++)
    {
        Pws[i].z = Pws_default[i].z + height;
    }
}

/*
 *@brief 设置机器人身体位置
 *@param body_pos 机器人的身体位置
 */
void Gait_prg::set_body_position(Position3 &body_pos)
{
    this->body_pos = body_pos;
    for (int i = 0; i < 6; i++)
    {
        Pws[i] = Pws_default[i] - body_pos;
    }
}

/*
 *@brief 设置机器人速度
 *@param velocity 机器人速度
 */
void Gait_prg::set_velocity(Velocity &velocity)
{
    this->velocity = velocity;
}

/*
 * 计算圆心位置和步伐大小已及步伐执行时间
 */
void Gait_prg::CEN_and_pace_cal()
{
    // 数据预处理，避免出现0
    if (velocity.Vx == 0)
        velocity.Vx += 0.001f;
    if (velocity.Vy == 0)
        velocity.Vy += 0.001f;
    if (velocity.omega == 0)
        velocity.omega += 0.001f;
/*
    if (velocity.omega < 0)
    {
        velocity.Vx = -velocity.Vx;
        velocity.Vy = -velocity.Vy;
    }
*/
    // 计算圆心模长
    CEN.x = -K_CEN * velocity.Vy / velocity.omega;
    CEN.y =  K_CEN * velocity.Vx / velocity.omega;
    // 计算步伐大小
    float speed_sq = pow(velocity.Vx, 2) + pow(velocity.Vy, 2) + pow(velocity.omega, 2);
    float module_speed = powf(speed_sq, 0.5f); // 获取合成速度模长
    
    if (module_speed > MAX_SPEED) module_speed = MAX_SPEED;

    if (velocity.omega < 0) 
				R_pace = -KR_2 * module_speed; 
		else 
				R_pace = KR_2 * module_speed;

    // 4. 计算步伐时间
		if(!is_wave)
		{
			if (R_pace > MAX_R_PACE)
					this->pace_time = 1000 / (R_pace / MAX_R_PACE);
			else
					this->pace_time = 1000;
		}
    else
		{
					this->pace_time = 1000;
		}
}

/*
 * 步态规划
 */
void Gait_prg::gait_proggraming()
{
    Position3 Vec_CEN2leg_ends[6];    // 圆心到腿部末端的向量
    static float angle_off[6];        // 圆心与机械腿末端的夹角
    static float norm_CEN2legs[6];    // 圆心到机械腿末端的模长
    static float Rp_ratios[6];        // 各个机械腿步态规划的大小比例
    Position3 Vec_Leg_Start2CEN_s[6]; // 腿部起始端到圆心起始端的向量
	
		uint32_t logic_round = LegControl_round;
    if (velocity.Vy > 0) { 
        logic_round = (LegControl_round + N_POINTS / 2) % N_POINTS;
    }
		
    for (int i = 0; i < 6; i++)
    {
        Vec_CEN2leg_ends[i] = Pws[i] + P_legs[i] - CEN;                                         // 计算圆心到每个腿部末端的向量
        angle_off[i] = atan2(Vec_CEN2leg_ends[i].y, Vec_CEN2leg_ends[i].x);                     // 计算圆心与机械腿末端的夹角
        norm_CEN2legs[i] = sqrt(pow(Vec_CEN2leg_ends[i].x, 2) + pow(Vec_CEN2leg_ends[i].y, 2)); // 计算圆心与机械腿末端的模长
        Vec_Leg_Start2CEN_s[i] = CEN - P_legs[i];                                               // 计算腿部起始端到圆心起始端的向量
    }
    float max_norm_CEN2legs = 0;
    for (int i = 0; i < 6; i++)
        if (norm_CEN2legs[i] > max_norm_CEN2legs)
            max_norm_CEN2legs = norm_CEN2legs[i]; // 选出最大模长

    static float R_paces[6]; // 各个机械腿的步长
    for (int i = 0; i < 6; i++)
    {
        Rp_ratios[i] = norm_CEN2legs[i] / max_norm_CEN2legs; // 计算各个机械腿步态规划的大小比例
        R_paces[i] = Rp_ratios[i] * R_pace;                  // 计算各个机械腿步态的大小
    }
		float effective_R0 = R_paces[0];

		if (is_obstacle_mode) {
				// 仅在越障模式下，人为放大 1.5 倍步长用于计算摆角
				effective_R0 = R_paces[0] * 1.5f;
				
				// 【物理安全锁】防止大跨步超出机械极限（建议值 80-90，根据实测调整）
				if (abs(effective_R0) > 85.0f) {
						effective_R0 = (effective_R0 > 0) ? 85.0f : -85.0f;
				}
		}
		float d_theta = 2 * effective_R0 / norm_CEN2legs[0];
    //float d_theta = 2 * R_paces[0] / norm_CEN2legs[0]; // 计算机械腿走一步绕圆心拐的角度，随便拿一组数据来算就行
    float step_size = d_theta / (N_POINTS / 2);

    /*********先对腿1，3，5做步态规划***********/
    static float angle_t;   // 用于计算该点的角度
    static float y_temp;    // 用于计算z轴高度的临时变量
    static Position3 point; // 用于存储末端坐标点
    for (int i = 0; i < 5; i += 2)
    {
        if (logic_round < N_POINTS / 2) // 0-9, 画下半圆
        {
            angle_t = angle_off[i] + d_theta / 2 - step_size * logic_round;  // 计算这个点的角度
            point.x = Vec_Leg_Start2CEN_s[i].x + norm_CEN2legs[i] * cos(angle_t); // 计算这个点的x轴坐标(相对于机械腿起始端)
            point.y = Vec_Leg_Start2CEN_s[i].y + norm_CEN2legs[i] * sin(angle_t); // 计算这个点的y轴坐标(相对于机械腿起始端)
            point.z = Pws[i].z;                                                   // 动作前半部分贴着地面，故取站立时的z轴坐标
        }
        else // 10-19，画上半圆
        {
            float swing_p = (float)(logic_round- (N_POINTS / 2))/ (N_POINTS / 2); // 进度 0.0-1.0
            float h_max = 45.0f; // 抬腿高度 35mm

            if (is_gate_test) 
            {

                if (swing_p < 0.2f) { // 阶段 1: 垂直上升
										angle_t = angle_off[i] - d_theta / 2; // 锁定在起点
                    point.z = Pws[i].z + (swing_p / 0.2f) * h_max;
								}   
                else if (swing_p < 0.8f) { // 阶段 2: 水平跨越
                    point.z = Pws[i].z + h_max;
                    angle_t = angle_off[i] - d_theta / 2 + d_theta * ((swing_p - 0.2f) / 0.6f); // 重新分配跨步进度
                } 
                else { // 阶段 3: 垂直落下
                    point.z = Pws[i].z + h_max - ((swing_p - 0.8f) / 0.2f) * h_max;
                    angle_t = angle_off[i] + d_theta / 2; // 锁定在终点
                }
								point.x = Vec_Leg_Start2CEN_s[i].x + norm_CEN2legs[i] * cos(angle_t);
								point.y = Vec_Leg_Start2CEN_s[i].y + norm_CEN2legs[i] * sin(angle_t);
            }
						else
						{
								angle_t = angle_off[i] - d_theta / 2 + step_size * (logic_round - N_POINTS / 2); // 计算这个点的角度
								point.x = Vec_Leg_Start2CEN_s[i].x + norm_CEN2legs[i] * cos(angle_t);                 // 计算这个点的x轴坐标(相对于机械腿起始端)
								point.y = Vec_Leg_Start2CEN_s[i].y + norm_CEN2legs[i] * sin(angle_t);                 // 计算这个点的y轴坐标(相对于机械腿起始端)
								y_temp = -R_pace + (logic_round - N_POINTS / 2) * (R_pace * 4 / N_POINTS);
								// 根据圆的大小缩小z轴高度,并迁移坐标系到机械腿末端,因为站立时z轴都是一样的，所以随便用一个Pw就行
								if(is_obstacle_mode == false)
								{
									if (R_pace > 0.5f && R_pace < MIN_Z_PACE)
											point.z = sqrt(pow(R_pace, 2) - pow(y_temp, 2)) * Rp_ratios[i] * 3 + Pws[i].z;
									else
											point.z = sqrt(pow(R_pace, 2) - pow(y_temp, 2)) * Rp_ratios[i] *1.5 + Pws[i].z;
								}
								else
								{
											point.z = sqrt(pow(R_pace, 2) - pow(y_temp, 2)) * Rp_ratios[i] * 4.5 + Pws[i].z;
								}
						}
						
        }
        point = hexapod_rotate(point, i);
        actions[i].thetas[LegControl_round] = ikine(point);
    }

    /*********对腿2，4，6做步态规划***********/
    for (int i = 1; i <= 5; i += 2)
    {
        if (logic_round < N_POINTS / 2) // 0-9, 画上半圆
        {
            float swing_p = (float)(logic_round)/ (N_POINTS / 2);
            float h_max = 45.0f; // 抬腿高度 35mm

           if (is_gate_test) 
            {
								
                if (swing_p < 0.2f) { // 阶段 1: 垂直上升
                    point.z = Pws[i].z + (swing_p / 0.2f) * h_max;
                    angle_t = angle_off[i] - d_theta / 2; // 锁定在起点
                } 
                else if (swing_p < 0.8f) { // 阶段 2: 水平跨越
                    point.z = Pws[i].z + h_max;
                    angle_t = angle_off[i] - d_theta / 2 + d_theta * ((swing_p - 0.2f) / 0.6f); // 重新分配跨步进度
                } 
                else { // 阶段 3: 垂直落下
                    point.z = Pws[i].z + h_max - ((swing_p - 0.8f) / 0.2f) * h_max;
                    angle_t = angle_off[i] + d_theta / 2; // 锁定在终点
                }
								point.x = Vec_Leg_Start2CEN_s[i].x + norm_CEN2legs[i] * cos(angle_t);
								point.y = Vec_Leg_Start2CEN_s[i].y + norm_CEN2legs[i] * sin(angle_t);
            }
						else
						{
								angle_t = angle_off[i] - d_theta / 2 + step_size * logic_round; // 计算这个点的角度
								point.x = Vec_Leg_Start2CEN_s[i].x + norm_CEN2legs[i] * cos(angle_t);                 // 计算这个点的x轴坐标(相对于机械腿起始端)
								point.y = Vec_Leg_Start2CEN_s[i].y + norm_CEN2legs[i] * sin(angle_t);                 // 计算这个点的y轴坐标(相对于机械腿起始端)
								y_temp = -R_pace + logic_round * (R_pace * 4 / N_POINTS);
								// 根据圆的大小缩小z轴高度,并迁移坐标系到机械腿末端,因为站立时z轴都是一样的，所以随便用一个Pw就行
								if(is_obstacle_mode == false)
								{
									if (R_pace > 0.5f && R_pace < MIN_Z_PACE)
											point.z = sqrt(pow(R_pace, 2) - pow(y_temp, 2)) * Rp_ratios[i] * 3 + Pws[i].z;
									else
											point.z = sqrt(pow(R_pace, 2) - pow(y_temp, 2)) * Rp_ratios[i] *1.5 + Pws[i].z;
								}
								else
								{
											point.z = sqrt(pow(R_pace, 2) - pow(y_temp, 2)) * Rp_ratios[i] * 4.5 + Pws[i].z;
								}
						}
						
        }
        else // 10-19, 画下半圆
        {
            angle_t = angle_off[i] + d_theta / 2 - step_size * (logic_round - N_POINTS / 2); // 计算这个点的角度
            point.x = Vec_Leg_Start2CEN_s[i].x + norm_CEN2legs[i] * cos(angle_t);                 // 计算这个点的x轴坐标(相对于机械腿起始端)
            point.y = Vec_Leg_Start2CEN_s[i].y + norm_CEN2legs[i] * sin(angle_t);                 // 计算这个点的y轴坐标(相对于机械腿起始端)
            point.z = Pws[i].z;
        }
        point = hexapod_rotate(point, i);
        actions[i].thetas[LegControl_round] = ikine(point);
    }  
}

uint32_t Gait_prg::get_pace_time()
{
    return this->pace_time;
}

void Gait_prg::run_wave_gait()
{
    // 1. 调用原有的圆心计算函数，这样 Vx, Vy, Omega 就都进来了
    this->CEN_and_pace_cal(); 

    float normalized_pt = (float)LegControl_round / (float)N_POINTS;
    
    // 2. 预计算所有腿相对于圆心的参数 (复用你三角步态的数学模型)
    Position3 Vec_CEN2leg_ends[6];
    float angle_off[6];
    float norm_CEN2legs[6];
    Position3 Vec_Leg_Start2CEN_s[6];

    for (int i = 0; i < 6; i++) {
        Vec_CEN2leg_ends[i] = Pws[i] + P_legs[i] - CEN;
        angle_off[i] = atan2(Vec_CEN2leg_ends[i].y, Vec_CEN2leg_ends[i].x);
        norm_CEN2legs[i] = sqrt(pow(Vec_CEN2leg_ends[i].x, 2) + pow(Vec_CEN2leg_ends[i].y, 2));
        Vec_Leg_Start2CEN_s[i] = CEN - P_legs[i];
    }

    // 3. 计算这一步总共要转过的弧度 (d_theta)
    // R_pace 是在 CEN_and_pace_cal 中算出来的步幅
    float d_theta = (norm_CEN2legs[0] > 1.0f) ? (2.0f * R_pace / norm_CEN2legs[0]) : 0;
    const float lift_height = 50.0f; 

		int wave_order[6] = {5, 4, 3, 0, 1, 2};
    for (int i = 0; i < 6; i++) 
    {
				int leg = wave_order[i]; // 获取当前应该动作的腿索引
        // 波浪步态相位：每条腿占 1/6 周期
        float phase_start = (float)i / 6.0f;
        float phase_end   = (float)(i + 1) / 6.0f;
        float swing_width = 1.0f / 6.0f;

        Position3 p_target;
        float angle_t;

        if (normalized_pt >= phase_start && normalized_pt < phase_end) 
        {
            // --- 摆动相 (Swing) ---
            float local_phase = (normalized_pt - phase_start) / swing_width;
            // 摆动时：从起始角度 -d_theta/2 摆动到 +d_theta/2
            angle_t = angle_off[leg] - d_theta/2.0f + d_theta * local_phase;
            
            p_target.x = Vec_Leg_Start2CEN_s[leg].x + norm_CEN2legs[leg] * cos(angle_t);
            p_target.y = Vec_Leg_Start2CEN_s[leg].y + norm_CEN2legs[leg] * sin(angle_t);
            p_target.z = Pws[leg].z + lift_height * sin(local_phase * PI);
        } 
        else 
        {
            // --- 支撑相 (Support) ---
            float rel = (normalized_pt >= phase_end) ? (normalized_pt - phase_end) : (normalized_pt + (1.0f - phase_end));
            float support_phase = rel / (1.0f - swing_width);
            
            // 支撑时：从 +d_theta/2 缓慢回到 -d_theta/2
            angle_t = angle_off[leg] + d_theta/2.0f - d_theta * support_phase;
            
            p_target.x = Vec_Leg_Start2CEN_s[leg].x + norm_CEN2legs[leg] * cos(angle_t);
            p_target.y = Vec_Leg_Start2CEN_s[leg].y + norm_CEN2legs[leg] * sin(angle_t);
            p_target.z = Pws[leg].z;
        }

        // 叠加机身旋转和机身位置偏置
        p_target = hexapod_rotate(p_target, leg);
        
        // 存入当前回合的动作序列
        actions[leg].thetas[LegControl_round] = ikine(p_target);
    }
}
