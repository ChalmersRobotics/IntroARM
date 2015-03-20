/*
 *	mpu6050.h
 *
 *	Created on: 19 mar 2015
 *		Author: Sterna and PŒsse
 */
#ifndef MPU6050_H_
#define MPU6050_H_

#define MPU_TIMEOUT_MS	100

void mpu6050Init();
void mpu6050Process();


extern volatile int16_t IMUValX;
extern volatile int16_t IMUValY;
extern volatile int16_t IMUValRoll;
extern volatile int16_t IMUValZ;
extern volatile int16_t IMUValPitch;
extern volatile int16_t IMUValYaw;

#endif /* MPU6050_H_ */
