#include "bno085.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

static uint8_t tx[32];

static float q14(int16_t v) {
	return ((float) v) / 16384.0f;
}

static float q8(int16_t v) {
	return ((float) v) / 256.0f;
}

static void quaternionToEuler(BNO085 *imu) {
	float x = imu->qx;
	float y = imu->qy;
	float z = imu->qz;
	float w = imu->qw;

	imu->roll = atan2f(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
	imu->pitch = asinf(2 * (w * y - z * x));
	imu->yaw = atan2f(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
}

static void sendPacket(BNO085 *imu, uint8_t channel, uint8_t *data,
		uint16_t len) {
	uint16_t plen = len + 4;

	/* clear continuation bit */
	plen &= 0x7FFF;

	tx[0] = plen & 0xFF;
	tx[1] = (plen >> 8) & 0x7F;

	tx[2] = channel;
	tx[3] = imu->seq[channel]++;

	memcpy(&tx[4], data, len);

	HAL_I2C_Master_Transmit(imu->i2c, BNO_ADDR, tx, plen, 100);
}

static void setFeature(BNO085 *imu, uint8_t report, uint32_t interval) {
	uint8_t cmd[17];
	memset(cmd, 0, sizeof(cmd));

	cmd[0] = 0xFD;        // Set Feature Command
	cmd[1] = report;      // Report ID
	cmd[2] = 0x00;        // Feature flags
	cmd[3] = 0x00;
	cmd[4] = 0x00;

	cmd[5] = interval & 0xFF;
	cmd[6] = (interval >> 8) & 0xFF;
	cmd[7] = (interval >> 16) & 0xFF;
	cmd[8] = (interval >> 24) & 0xFF;

	sendPacket(imu, 2, cmd, 17);
}

void BNO085_EnableRotationVector(BNO085 *imu, uint32_t interval_us) {
	setFeature(imu, 0x05, interval_us);
}

void BNO085_EnableLinearAccel(BNO085 *imu, uint32_t interval_us) {
	setFeature(imu, 0x04, interval_us);
}

void BNO085_Init(BNO085 *imu, I2C_HandleTypeDef *hi2c, GPIO_TypeDef *intPort, uint16_t intPin, GPIO_TypeDef *rstPort, uint16_t rstPin) {
	imu->i2c = hi2c;
	imu->i2c = hi2c;
	imu->intPort = intPort;
	imu->intPin = intPin;
	imu->rstPort = rstPort;
	imu->rstPin = rstPin;

	memset(imu->seq, 0, 6);
	HAL_GPIO_WritePin(imu->rstPort, imu->rstPin, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(imu->rstPort, imu->rstPin, GPIO_PIN_SET);
	HAL_Delay(600);

}

void BNO085_Process(BNO085 *imu, volatile uint8_t *dataReadyFlag) {
	if (*dataReadyFlag) {
		*dataReadyFlag = 0; // Clear the flag

		/* Drain the queue */
		for (uint8_t n = 0; n < 8; n++) {
			// Check if the pin is still low using the struct's GPIO mapping
			if (HAL_GPIO_ReadPin(imu->intPort, imu->intPin) != GPIO_PIN_RESET) {
				break;
			}

			if (BNO085_ReadPacket(imu) != HAL_OK) {
				break;
			}
		}
	}
}
static void parse(BNO085 *imu, uint8_t *payload, uint16_t payloadLen) {
	// SHTP Channel 3 payload ALWAYS starts with a 5-byte timestamp.
	if (payloadLen < 5)
		return;

	uint16_t i = 5; // Start parsing after the 5-byte timestamp

	while (i < payloadLen) {
		uint8_t id = payload[i];

		if (id == 0x05) { // Rotation Vector (14 bytes total)
			if (i + 14 > payloadLen)
				break;

			// Cast to int16_t first to handle negative values properly
			imu->qx = q14((int16_t) (payload[i + 4] | (payload[i + 5] << 8)));
			imu->qy = q14((int16_t) (payload[i + 6] | (payload[i + 7] << 8)));
			imu->qz = q14((int16_t) (payload[i + 8] | (payload[i + 9] << 8)));
			imu->qw = q14((int16_t) (payload[i + 10] | (payload[i + 11] << 8)));

			quaternionToEuler(imu);
			i += 14;
		} else if (id == 0x04) { // Linear Accel (10 bytes total)
			if (i + 10 > payloadLen)
				break;

			imu->ax = q8((int16_t) (payload[i + 4] | (payload[i + 5] << 8)));
			imu->ay = q8((int16_t) (payload[i + 6] | (payload[i + 7] << 8)));
			imu->az = q8((int16_t) (payload[i + 8] | (payload[i + 9] << 8)));

			i += 10;
		} else if (id == 0x01 || id == 0x02 || id == 0x03) {
			// Standard Accel, Gyro, Mag are all 10 bytes. Skip them if we aren't saving them.
			i += 10;
		} else {
			// Unknown report ID encountered. Break to avoid infinite loops or parsing misaligned data.
			break;
		}
	}
}

HAL_StatusTypeDef BNO085_ReadPacket(BNO085 *imu) {
	static uint8_t packet[300]; // Buffer for the entire packet
	HAL_StatusTypeDef status;

	// 1. Read just the first 4 bytes to determine the packet length
	status = HAL_I2C_Master_Receive(imu->i2c, BNO_ADDR, packet, 4, 100);
	if (status != HAL_OK)
		return status;

	uint16_t len = ((uint16_t) packet[1] << 8) | packet[0];
	len &= 0x7FFF;

	// Reject bad packets or packets larger than our buffer
	if (len < 4 || len > sizeof(packet)) {
		return HAL_ERROR;
	}

	// 2. Read the ENTIRE packet.
	// Because we start a new I2C transaction, the BNO085 will resend the header + payload.
	status = HAL_I2C_Master_Receive(imu->i2c, BNO_ADDR, packet, len, 100);
	if (status != HAL_OK)
		return status;

	uint8_t channel = packet[2];
	uint8_t seq = packet[3];

	// Synchronize host sequence counter
	imu->seq[channel] = seq;

	// 3. If it's Sensor Hub Data (Channel 3), pass the payload to the parser
	if (channel == 3) {
		// packet[0..3] is header, packet[4..len-1] is the actual payload
		parse(imu, &packet[4], len - 4);
	}

	return HAL_OK;
}
