#include "MPU_task.h"
#include "mpu6050.h"
#include "cmsis_os.h"
#include "debug_uart.h"

extern MPU_6050 mpu6050;

extern "C"
{
  void MPU_Task(void const *argument)
  {
    osDelay(100);
    mpu6050.Init();
    osDelay(3000); //µÈ´ýÍÓÂÝÒÇ³õÊ¼»¯
		mpu6050.Calibrate_Gyro();
    while (1)
    {
      mpu6050.dmp_get_data();
			mpu6050.Read_Gyro();
			//APP_PRINT("MPU: Pitch=%.2f,Roll=%.2f,Yaw=%.2f,GX=%.2f,GY=%.2f \r\n", mpu6050.angle.x, mpu6050.angle.y, mpu6050.angle.z, mpu6050.gyro_accel.x, mpu6050.gyro_accel.y);
			//APP_PRINT("GX=%.2f,GY=%.2f \r\n", mpu6050.gyro_accel.x, mpu6050.gyro_accel.y);
      osDelay(10);
    }
  }
}
