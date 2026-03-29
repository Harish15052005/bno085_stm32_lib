#ifndef BNO085_H
#define BNO085_H

#include "stm32f1xx_hal.h"

#define BNO_ADDR (0x4A << 1)

typedef struct {
	I2C_HandleTypeDef *i2c;

	// Hardware pins
	GPIO_TypeDef *intPort;
	uint16_t intPin;
	GPIO_TypeDef *rstPort;
	uint16_t rstPin;

	uint8_t seq[6];

	float ax, ay, az;

	float qx, qy, qz, qw;

	float roll, pitch, yaw;

} BNO085;

void BNO085_Init(BNO085 *imu, I2C_HandleTypeDef *hi2c, GPIO_TypeDef *intPort, uint16_t intPin, GPIO_TypeDef *rstPort, uint16_t rstPin);
void BNO085_EnableRotationVector(BNO085 *imu, uint32_t interval_us);

void BNO085_EnableLinearAccel(BNO085 *imu, uint32_t interval_us);

void BNO085_Process(BNO085 *imu, volatile uint8_t *dataReadyFlag);
HAL_StatusTypeDef BNO085_ReadPacket(BNO085 *imu);

#endif
