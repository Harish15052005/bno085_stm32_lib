Library to interface bno085 imu sensor with stm32 boards. 
The library uses i2c and interrupts to read sensors values. 

Currentlly only linear acceleration and rotaiton vectors are request from the sensor. 

Usage:
Connect reset, interrupt scl, sda, vin and gnd. 
Enable nvic interrupt with falling edge detection. 

Example code is provided in the main.c file under Core/Src/main.c
