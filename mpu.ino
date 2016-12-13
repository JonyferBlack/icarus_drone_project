//mpu
	void getYPR()
	{
	//	bool mpuIntStatus = mpu.checkDataReady();

	//	if (mpuIntStatus)
//		{
			//mpu.readAccelXYZ(&ypr[0], &ypr[1], &ypr[2]);
	//	}
	}

	inline void dmpDataReady() 
	{
	}

	void initMPU()
	{
		Wire.begin();
		Serial.println(F("Trying to init MPU"));
		if (!mpu.searchDevice()) return;
		mpu.begin();
		mpu.configMPU9250();
		Serial.println(F("Mpu ready"));
	}