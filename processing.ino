void processing(){
 Serial.print(rc_throttle);
 delay(2);
 Serial.print(" ");
 Serial.print(pwm_FL, 0);
 Serial.print(" ");
 Serial.print(pwm_FR, 0);
 Serial.print(" ");
 Serial.print(pwm_BL, 0);
 Serial.print(" ");
 Serial.print(pwm_BR, 0);
 Serial.print(" ");
 Serial.print(gyro_pitch, 0);
 Serial.print(" ");
 Serial.print(gyro_roll, 0);
 Serial.print(" ");
 Serial.print(mpu_pitch);
 Serial.print(" ");
 Serial.print(mpu_roll); 
 Serial.print(" ");
 Serial.print(ackPayload.dial1);
 Serial.print(" ");
 Serial.print(ackPayload.dial2, 3);
 Serial.println();
  
}

