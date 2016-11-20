#include <Servo.h>
#include <Wire.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <FaBo9Axis_MPU9250.h>

namespace SmartDroneProject {
	//MPU
	FaBo9Axis mpu;                           // mpu interface object
	float ypr[3] = { 0.0f,0.0f,0.0f };       // yaw pitch roll values

	Servo FL, FR, BL, BR;

	float sampletime;
	int rc_throttle;
	int rc_input_limit;
	float pwm_FL, pwm_FR, pwm_BL, pwm_BR;
	float rc_yaw, rc_pitch, rc_roll;
	float mpu_yaw, mpu_pitch, mpu_roll;

	float gyro_yaw, gyro_pitch, gyro_roll;
	float gx, gy, gz; //Gyro values
	//MPU6050 accelgyro;

	void setup()
	{
		FL.attach(3, 1000, 2000);
		FR.attach(4, 1000, 2000);
		BL.attach(5, 1000, 2000);
		BR.attach(6, 1000, 2000);

		Serial.begin(115200);
		initializeRX();
		initMPU();

		mpu_yaw = 0.0;
		rc_yaw = 0.0;
		mpu_pitch = 0.0;
		rc_pitch = 0.0;
		mpu_roll = 0.0;
		rc_roll = 0.0;
		rc_input_limit = 30;
	}



	void loop()
	{

		readRX();

		mpu.readGyroXYZ(&gx, &gy, &gz);
		gyro_yaw = -gz / 16.4;
		gyro_pitch = -gy / 16.4;
		gyro_roll = gx / 16.4;

		getYPR();
		mpu_yaw = ypr[0] * 180 / M_PI;
		mpu_pitch = ypr[1] * 180 / M_PI;
		mpu_roll = ypr[2] * 180 / M_PI;


		//debug();
		processing();

		FL.writeMicroseconds(pwm_FL);
		FR.writeMicroseconds(pwm_FR);
		BL.writeMicroseconds(pwm_BL);
		BR.writeMicroseconds(pwm_BR);

	}
}