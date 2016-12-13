#include <Servo.h>
#include <Wire.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <FaBo9Axis_MPU9250.h>
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

		ResetEngines();

		Serial.begin(115200);
		initializeRX();
		initMPU();
	}

	

	const int LowEdge = 1050;
	const int HighEdge = 2000;
	const long TimeIntervalMillisecs = 100;

	int i = LowEdge;

	bool _motionAllowed = true;
	unsigned long _previousMillis = 0;

	void loop()
	{
		if (Serial.available() > 0)
		{
			int flag = Serial.parseInt();
			_motionAllowed = flag != 0;
			Serial.print("Specified flag: ");
			Serial.println(flag);
		}
//		readRX();

//		mpu.readGyroXYZ(&gx, &gy, &gz);

		//debug();
//		processing();

		if (_motionAllowed) {
			RotateEngines();
		}
		else
		{
			ResetEngines();
		}
	}

	void RotateEngines() {
	
		unsigned long currentMillis = millis();

		if (currentMillis - _previousMillis >= TimeIntervalMillisecs) {
			// save the last time you blinked the LED
			_previousMillis = currentMillis;

			if (i >= HighEdge) i = LowEdge;

			FL.writeMicroseconds(i);
			FR.writeMicroseconds(i);;
			BL.writeMicroseconds(i);
			BR.writeMicroseconds(i);
			Serial.println(i);
			Serial.println(i * 10);
			i++;
		}

		//ResetEngines();
	}

	void ResetEngines() {
		FL.write(0);
		FR.write(0);;
		BL.write(0);
		BR.write(0);
		Serial.println("Engines reset");
	}