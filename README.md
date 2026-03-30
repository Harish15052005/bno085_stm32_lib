Library to interface bno085 imu sensor with stm32 boards. 
The library uses i2c and interrupts to read sensors values. 

Currentlly only linear acceleration and rotaiton vectors are request from the sensor. 

Usage:
Connect reset, interrupt scl, sda, vin and gnd. 
Enable nvic interrupt with falling edge detection. 

Example code is provided in the main.c file under Core/Src/main.c

Library is currently tested on bluepill boards(stm32f103c8t6)

# SETUP STEPS
1. setup i2c. 
2. setup nvic interrupt with falling edge detection for bno085 interrupt pin.
3. setup a gpio as output pin to reset bno085 sensor.
4. copy the bno085.h file to /Core/Inc/ and bno085.c file to /Core/Src/
5. add following code in `HAL_GPIO_EXTI_Callback` function:
`if (GPIO_Pin == GPIO_PIN_1) {
		imuDataReady = 1;
	}`
    (**note**: change `GPIO_PIN_1` to the pin you are using for interrupt)
    (Initialize `imuDataReady` as a global variable of type `volatile uint8_t` (line 53 in example.c) and set it to 0)
    (line 97 in example.c)
6. make an instance of sensor `BNO085` (line 51 in example.c);
7. initialize the sensor with `BNO085_Init` function (line 141 in example.c);
8. add following after that (This is to drain startup packets) (line 143 in example.c):
```
uint32_t start = HAL_GetTick();
	while (HAL_GetTick() - start < 1000) {
		if (imuDataReady) {
			imuDataReady = 0;
			BNO085_ReadPacket(&imu);
		}
	}
```
9. Enable reports you want to read. (line 151 in example.c)
(**note**: currently only rotationVecotrs and linear acceleration are implemented)
10. Call BNO085_Process function in main loop. (line 170 in example.c). Pass the pointer to sensor instance and pointer to the imuDataReady variable. This will read the data from the sensor and update the sensor instance variables.
11. The data avaiable in sensor instances according below structure:
```
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
```

