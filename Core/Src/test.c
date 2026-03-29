//#include "main.h"
//#include "bno085.h"
//
//extern I2C_HandleTypeDef hi2c2;
//
//BNO085 imu;
//
//int main(void)
//{
//
//HAL_Init();
//
//SystemClock_Config();
//
//MX_GPIO_Init();
//MX_I2C2_Init();
//
//BNO085_Init(&imu,&hi2c2);
//
//BNO085_EnableRotationVector(&imu,10000);
//BNO085_EnableLinearAccel(&imu,10000);
//
//while(1)
//{
//
//BNO085_ReadPacket(&imu);
//
//float roll=imu.roll;
//float pitch=imu.pitch;
//float yaw=imu.yaw;
//
//float ax=imu.ax;
//float ay=imu.ay;
//float az=imu.az;
//
//HAL_Delay(10);
//
//}
//}
