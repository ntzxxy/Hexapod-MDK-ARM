#include "mpu6050.h"
#include "inv_mpu.h"
#include "my_math.h"

MPU_6050 mpu6050;

void MPU_6050::Init()
{
    MX_I2C4_Init();
    this->hi2c = &hi2c4;
    // this->I2C_Write(MPU6050_RA_PWR_MGMT_1, 0x00); // 解除休眠状态
    // this->set_gyro_sampling_fre(200);             // 设置陀螺仪采样率
    // this->I2C_Write(MPU6050_RA_CONFIG, 0x06);
    // this->I2C_Write(MPU6050_RA_ACCEL_CONFIG, 0x01); // 配置加速度传感器工作在4G模式
    // this->I2C_Write(MPU6050_RA_GYRO_CONFIG, 0x18);  // 陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
    atk_ms6050_dmp_init();                          // 初始化dmp
}

/*
 *@brief I2C发送一个字节
 *@param
 */
void MPU_6050::I2C_Write(uint16_t MemAddress, uint8_t data)
{
    HAL_I2C_Mem_Write(this->hi2c, MPU_DEFAULT_ID, MemAddress, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);
}
/*
 *@brief I2C发送一个字符串
 *@param
 */
void MPU_6050::I2C_Write(uint16_t MemAddress, uint8_t *str, uint8_t str_len)
{
    HAL_I2C_Mem_Write(this->hi2c, MPU_DEFAULT_ID, MemAddress, I2C_MEMADD_SIZE_8BIT, str, str_len, 1000);
}

void MPU_6050::I2C_Read(uint16_t MemAddress, uint8_t *str, uint8_t str_len)
{
    HAL_I2C_Mem_Read(this->hi2c, MPU_DEFAULT_ID, MemAddress, I2C_MEMADD_SIZE_8BIT, str, str_len, 1000);
}

// 软件复位
void MPU_6050::sw_reset()
{
    this->I2C_Write(MPU6050_RA_PWR_MGMT_1, 0x80);
}

void MPU_6050::Read_Gyro()
{
    this->I2C_Read(MPU6050_GYRO_OUT, receive_buffer, 6);
    // 1. 原始转换 (int16_t 强转解决 65535 问题)
    float raw_gx = (float)((int16_t)(receive_buffer[0] << 8 | receive_buffer[1])) / 16.4f;
    float raw_gy = (float)((int16_t)(receive_buffer[2] << 8 | receive_buffer[3])) / 16.4f;
    float raw_gz = (float)((int16_t)(receive_buffer[4] << 8 | receive_buffer[5])) / 16.4f;

    // 2. 扣除零点偏移
    this->gyro_accel.x = raw_gx - this->gyro_offset.x;
    this->gyro_accel.y = raw_gy - this->gyro_offset.y;
    this->gyro_accel.z = raw_gz - this->gyro_offset.z;
}

Position3 MPU_6050::get_angle()
{
		Position3 angle_rad(this->angle.x/180*PI, this->angle.y/180*PI,this->angle.z/180*PI); //转换为弧度制
    return angle_rad;
}

/*
 *@brief 设置采样率
 *@param fre 采样率 4-1000Hz
 */
void MPU_6050::set_gyro_sampling_fre(uint32_t fre)
{
    if (fre > 1000)
        fre = 1000;
    if (fre < 4)
        fre = 4;

    uint32_t sample_division = 1000 / fre - 1;
    this->I2C_Write(MPU6050_RA_SMPLRT_DIV, sample_division);
}

void MPU_6050::dmp_get_data()
{
    atk_ms6050_dmp_get_data(&(this->angle.x), &(this->angle.y), &(this->angle.z));  //换算为弧度制
}

void MPU_6050::Calibrate_Gyro() {
    float sum_x = 0, sum_y = 0, sum_z = 0;
    const int sample_num = 200; // 采集200次求平均值

    for (int i = 0; i < sample_num; i++) {
        this->Read_Gyro(); // 此时 Read_Gyro 内部还没减去 offset
        sum_x += this->gyro_accel.x;
        sum_y += this->gyro_accel.y;
        sum_z += this->gyro_accel.z;
        HAL_Delay(5); // 稍微延时
    }
    
    // 计算平均偏移量
    this->gyro_offset.x = sum_x / (float)sample_num;
    this->gyro_offset.y = sum_y / (float)sample_num;
    this->gyro_offset.z = sum_z / (float)sample_num;
}
