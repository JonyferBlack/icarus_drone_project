namespace SmartDroneProject {
	
	const uint64_t pipeIn = 0xE8E8F0F0E1LL;
	RF24 radio(9, 10);

	// The sizeof this struct should not exceed 32 bytes
	struct RF24Data {
		byte rc_throttle;
		byte yaw;
		byte pitch;
		byte roll;
		byte dial1;
		byte dial2;
		byte flag;
	};

	struct RF24AckPayload {
		int16_t rc_throttle;
		int16_t pwm_FL;
		int16_t pwm_FR;
		int16_t pwm_BL;
		int16_t pwm_BR;
		int16_t yaw;
		int16_t pitch;
		int16_t roll;
		int16_t gyro_yaw;
		int16_t gyro_pitch;
		int16_t gyro_roll;
		float dial1;
		float dial2;
	};

	RF24Data nrf24Data;
	RF24AckPayload ackPayload;

	void resetData()
	{
		// 'safe' values to use when no radio input is detected
		nrf24Data.rc_throttle = 0;
		nrf24Data.yaw = 127;
		nrf24Data.pitch = 127;
		nrf24Data.roll = 127;
		nrf24Data.dial1 = 0;
		nrf24Data.dial2 = 0;
		nrf24Data.flag = '3';
		map_receivedData();
	}

	void resetRF24AckPayload()
	{
		ackPayload.rc_throttle = 0;
		ackPayload.pwm_FL = 0;
		ackPayload.pwm_FR = 0;
		ackPayload.pwm_BL = 0;
		ackPayload.pwm_BR = 0;
		ackPayload.pitch = 0;
		ackPayload.roll = 0;
		ackPayload.gyro_pitch = 0;
		ackPayload.gyro_roll = 0;
		ackPayload.dial1 = 0.0;
		ackPayload.dial2 = 0.0;
	}


	void initializeRX()
	{
		resetData();
		resetRF24AckPayload();
		radio.begin(); // Set up radio module
		radio.setDataRate(RF24_250KBPS); // Both endpoints must have this set the same
		radio.setAutoAck(1);
		radio.enableAckPayload();
		radio.openReadingPipe(1, pipeIn);
		radio.startListening();
	}


	void map_receivedData() {

		rc_throttle = map(nrf24Data.rc_throttle, 0, 255, 1000, 2000);
		rc_yaw = map(nrf24Data.yaw, 0, 255, -150, 150);
		rc_pitch = map(nrf24Data.pitch, 0, 255, -rc_input_limit, rc_input_limit);
		rc_roll = map(nrf24Data.roll, 0, 255, -rc_input_limit, rc_input_limit);

	}

	unsigned long lastRecvTime = 0;

	void readRX()
	{
		ackPayload.rc_throttle = rc_throttle;
		ackPayload.pwm_FL = pwm_FL;
		ackPayload.pwm_FR = pwm_FR;
		ackPayload.pwm_BL = pwm_BL;
		ackPayload.pwm_BR = pwm_BR;
		ackPayload.yaw = mpu_yaw;
		ackPayload.pitch = mpu_pitch;
		ackPayload.roll = mpu_roll;
		ackPayload.gyro_yaw = gyro_yaw;
		ackPayload.gyro_pitch = gyro_pitch;
		ackPayload.gyro_roll = gyro_roll;


		//Read Radio Data
		while (radio.available()) {        //as long as radio data is available
			radio.writeAckPayload(1, &ackPayload, sizeof(RF24AckPayload));
			radio.read(&nrf24Data, sizeof(nrf24Data));
			lastRecvTime = millis();  //set time of last received data to current time
		}

		// signal lost?
		unsigned long now = millis();
		if (now - lastRecvTime > 1000) {
			resetData();
		}

		rc_throttle = map(nrf24Data.rc_throttle, 0, 255, 1000, 2000);
		rc_yaw = map(nrf24Data.yaw, 0, 255, -40, 40);
		rc_pitch = map(nrf24Data.pitch, 0, 255, -rc_input_limit, rc_input_limit);
		rc_roll = map(nrf24Data.roll, 0, 255, -rc_input_limit, rc_input_limit);
	}
}





