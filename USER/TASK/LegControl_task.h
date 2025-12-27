#ifndef LEGCONTROL_TASK_H
#define LEGCONTROL_TASK_H

#include "my_math.h"
#include "leg.h"
#include "remote.h"
#include "gait_prg.h"

#define LEG_JOINT2_OFFSET 0
#define LEG_JOINT3_OFFSET 0

#define HEXAPOD_MIN_HEIGHT -10.0f
#define HEXAPOD_MAX_HEIGHT 10.0f
#define HEXAPOD_MIN_X -40.0f
#define HEXAPOD_MAX_X 40.0f
#define HEXAPOD_MIN_Y -40.0f
#define HEXAPOD_MAX_Y 40.0f


#define HEXAPOD_MIN_X_ROTATE -15.0f / 180 * PI // 绕X轴旋转角度最小为-15度
#define HEXAPOD_MAX_X_ROTATE 15.0f / 180 * PI  // 绕X轴旋转角度最大为 15度
#define HEXAPOD_MIN_Y_ROTATE -10.0f / 180 * PI // 绕X轴旋转角度最小为-10度
#define HEXAPOD_MAX_Y_ROTATE 10.0f / 180 * PI  // 绕X轴旋转角度最大为 10度
#define HEXAPOD_MIN_Z_ROTATE -25.0f / 180 * PI // 绕X轴旋转角度最小为-25度
#define HEXAPOD_MAX_Z_ROTATE 25.0f / 180 * PI  // 绕X轴旋转角度最大为 25度

/*PID*/
#define MPU_X_PID_KP 0.015f
#define MPU_X_PID_KI 0.0f
#define MPU_X_PID_KD 0.5f

#define MPU_Y_PID_KP 0.015f
#define MPU_Y_PID_KI 0.0f
#define MPU_Y_PID_KD 0.5f


/*FOF一阶低通滤波参数*/
#define VELOCITY_FOF_K 0.06f
#define BODY_POS_FOF_K 0.1f
#define BODY_ANGLE_FOF_K 0.1f


#define ROTATE_BODY_ANGLE_SENSI 0.00002f//控制角度灵敏度
#define ROTATE_BODY_POS_SENSI 0.006f//控制位置灵敏度


typedef enum
{
    HEXAPOD_MOVE,
    HEXAPOD_BODY_ANGEL_CONTROL,
    HEXAPOD_BODY_POS_CONTROL,
		HEXAPOD_LOCK,
} Hexapod_mode_e;

typedef enum
{
    MPU_ON,
    MPU_OFF,
}MPU_SW_e;

typedef enum {
    MODE_IDLE,          
    MODE_IK_TEST,
		MODE_GAIT_RUN,	
    MODE_RESET_ZERO,    
    MODE_SINGLE_DEBUG,
		MODE_ACTION_TEACH
} ControlMode_t;

typedef enum {
    UI_MODE_MOVEMENT = 0,   // 运动控制模式 (行走、姿态)
    UI_MODE_TEACHING,       // 示教模式 (掉电、录制、上电)
    UI_MODE_DEBUG,          // 单腿调试模式 (可选)
		UI_MODE_REMOTE,
    UI_MODE_MAX             // 用于循环计数
} UIMode_t;

extern ControlMode_t current_mode;
extern Position3 target_pos;
extern UIMode_t current_ui_mode;

class Hexapod
{
public:
    Leg legs[6];                 // 六条腿
    Velocity velocity;           // 机器人速度
    Hexapod_mode_e mode; // 机器人模式
    MPU_SW_e mpu_sw;    //是否由陀螺仪控制
    Position3 body_pos;     //机体位置
    Position3 body_angle;   //机体角度
    Position3 mpu_angle;
    Position3 mpu_angle_set;
    PID mpu_pid_x; //x轴pid
    PID mpu_pid_y; //y轴pid
    First_order_filter velocity_fof[3];
    First_order_filter body_pos_fof[3];
    First_order_filter body_angle_fof[3];
    bool mpu_flag;
    void Init();
		void starting_pose();
    void velocity_cal(const RC_remote_data_t &remote_data);
    void body_position_cal(const RC_remote_data_t &remote_data);
    void body_angle_cal(const RC_remote_data_t &remote_data);
    void mode_select(const RC_remote_data_t &remote_data);
    void body_angle_and_pos_zero(const RC_remote_data_t &remote_data);
    void move(uint32_t round_time);

};

extern Hexapod hexapod;
extern uint8_t CALIB_LEG_INDEX;
extern Position3 calib_target_pos;
extern bool g_motor_power_on;
extern uint32_t LegControl_round;

#endif
