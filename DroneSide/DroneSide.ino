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
	int rc_input_limit;
	float pwm_FL, pwm_FR, pwm_BL, pwm_BR;
	float rc_yaw, rc_pitch, rc_roll;
	float mpu_yaw, mpu_pitch, mpu_roll;

	float gyro_yaw, gyro_pitch, gyro_roll;
	float gx, gy, gz; //Gyro values
	float ax, ay, az;
	float mx, my, mz;
	//MPU6050 accelgyro;

	const byte zeroPwm = 0;
	const int LowEdge = 1000;
	const int HighEdge = LowEdge + 1200;
	const long TimeIntervalMillisecs = 100;
	const int Start = 1250;
	const int End = Start + 100;

	int i = Start;

	int rc_throttle = 1250;

	void proccesCounter(int *i) {
		if (*i >= End) {
			*i = Start;
			resetEngines();
		}
	}

	bool _motionAllowed = true;
	unsigned long _previousMillis = 0;

	String s;
	byte message[5];

	void setup()
	{
		Serial.begin(115200);
		Serial.println("Begin setup");
		FL.attach(3, LowEdge, HighEdge);
		FR.attach(4, LowEdge, HighEdge);
		BL.attach(5, LowEdge, HighEdge);
		BR.attach(6, LowEdge, HighEdge);
		resetEngines();
		Serial.setTimeout(10);
		//		initializeRX();
		initMPU();
		Serial.println("Setup completed");
		//FL.writeMicroseconds(0); //ESC accepts value of speed and maintain this value by itself
	}

	void startCalibration() {
		for (int i = 0; i < 100; i++)
		{
			mpu.readGyroXYZ(&gx, &gy, &gz);
			mpu.readAccelXYZ(&ax, &ay, &az);
			mpu.readMagnetXYZ(&mx, &my, &mz);
		}
	}

	int _suspendsCount;

	void suspend() {
		_suspendsCount++;
	}

	void resume() {
		_suspendsCount--;
	}

	bool suspended() {
		return _suspendsCount > 0;
	}

	void loop()
	{
		if (mpu.checkDataReady() && !suspended()) {
			mpu.readGyroXYZ(&gx, &gy, &gz);
			mpu.readAccelXYZ(&ax, &ay, &az);
			mpu.readMagnetXYZ(&mx, &my, &mz);
		}
		
		rotateEngines();
	}

	void serialEvent() {
		suspend();
		s = Serial.readString();
		parseSerialData();
		resume();
	}

	void proccesMessage() {
		if (message[0] == 16)
		{
			//Detect Command type
			switch (message[1])
			{
			case 126:
			{
				Serial.print(gx);
				Serial.print("@");
				Serial.print(gy);
				Serial.print("@");
				Serial.print(gz);
				Serial.print("@");
				Serial.print(ax);
				Serial.print("@");
				Serial.print(ay);
				Serial.print("@");
				Serial.print(az);
				Serial.print("@");
				Serial.print(mx);
				Serial.print("@");
				Serial.print(my);
				Serial.print("@");
				Serial.println(mz);
				break;
			}
			case 127:
				//Say hello
				Serial.println("HELLO FROM ARDUINO");
				break;
			}
		}
	}

	void clearMessage() {
		for (int i = 0; i < 5; i++)
			message[i] = 0;
	}

	void fillMessage() {
		byte offset = s.indexOf(16);
		if (offset != -1 && s.length() >= offset + 5)
		{
			for (int i = 0; i < 5; i++) {
				message[i] = s.charAt(i + offset);
			}
		}
	}

	void parseSerialData() {
		fillMessage();
		proccesMessage();
		clearMessage();
	}

	unsigned long currentMillis;

	void rotateEngines() {
		
		if (!_motionAllowed) return;

		currentMillis = millis();

		if (currentMillis - _previousMillis >= TimeIntervalMillisecs) {
			_previousMillis = currentMillis;
			proccesCounter(&i);
			//writeEngines(&i);
			BL.writeMicroseconds(i);
			i++;
			_motionAllowed = false;
			//Serial.println("Engine speed set");
		}
	}

	void writeEngines(int *i) {
		FL.writeMicroseconds(*i);
		FR.writeMicroseconds(*i);;
		BL.writeMicroseconds(*i);
		BR.writeMicroseconds(*i);
		Serial.println(*i);
	}

	void resetEngines() {
		writeEngines(zeroPwm);
		Serial.println("Engines reset");
	}