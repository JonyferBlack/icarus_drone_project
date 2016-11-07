//mpu

void getYPR(){
  
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    
    if((mpuIntStatus & 0x10) || fifoCount >= 1024)
    { 
      mpu.resetFIFO(); 
    }
    else if(mpuIntStatus & 0x02)
    {
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      fifoCount -= packetSize;
    
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); 
    }

}




inline void dmpDataReady() {
    mpuInterrupt = true;
}


void initMPU(){
  
  Wire.begin();
  mpu.initialize();
  devStatus = mpu.dmpInitialize();

  //offset values were found by calibration 
  //if you use another sensor you need to run the calibration script again and change values here
  mpu.setXAccelOffset(-2161);
  mpu.setYAccelOffset(1217);
  mpu.setZAccelOffset(1596);
  mpu.setXGyroOffset(3);
  mpu.setYGyroOffset(4);
  mpu.setZGyroOffset(25);
  
mpu.setFullScaleGyroRange(3);
  
  if(devStatus == 0){
  
    mpu.setDMPEnabled(true);
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    packetSize = mpu.dmpGetFIFOPacketSize();
    
  }
}

