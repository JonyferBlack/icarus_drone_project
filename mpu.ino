//mpu
namespace SmartDroneProject {
	
	void getYPR()
	{
		bool mpuIntStatus = mpu.checkDataReady();

		if (mpuIntStatus)
		{
			mpu.readAccelXYZ(&ypr[0], &ypr[1], &ypr[2]);
		}
	}

	inline void dmpDataReady() 
	{
	}

	void initMPU()
	{
		Wire.begin();
		if (!mpu.searchDevice) return;
		mpu.begin();
	}
}
