//debug
void debug(){
 Serial.print(F("rc_throttle:"));
 Serial.print(rc_throttle);
 Serial.print('\t');
 Serial.print(F("rc_yaw:"));
 Serial.print(rc_yaw);
// Serial.print('\t');
// Serial.print(F("rcpit: "));
// Serial.print(rc_pitch, 0);
// Serial.print('\t');
// Serial.print(F("rc_roll: "));
// Serial.print(rc_roll, 0);
//Serial.print('\t');

//  Serial.print(F("stab_pitch_out:"));
//  Serial.print(stab_pitch_out);
//  Serial.print('\t');
//  Serial.print(F("stab_roll_out:"));
//  Serial.print(stab_roll_out);
//  Serial.print('\t');
//  Serial.print(F("pwm_FL:"));
//  Serial.print(pwm_FL, 0);
//  Serial.print('\t');
//  Serial.print(F("pwm_FR:"));
//  Serial.print(pwm_FR, 0);
//  Serial.print('\t');
//  Serial.print(F("pwm_BL:"));
//  Serial.print(pwm_BL, 0);
//  Serial.print('\t');
//  Serial.print(F("pwm_BR:"));
//  Serial.print(pwm_BR, 0);
//   
    Serial.print('\t');
    Serial.print(F("gyro_yaw: "));
    Serial.print(gyro_yaw, 0);
//  Serial.print('\t');
// Serial.print(F("gyro_pitch: "));
// Serial.print(gyro_pitch, 0);
// Serial.print('\t'); 
// Serial.print(F("gyro_roll: "));
// Serial.print(gyro_roll, 0);
 
  Serial.print('\t');
  Serial.print("yaw:");
  Serial.print(mpu_yaw, 0);
//  Serial.print('\t');
//  Serial.print(F("pit:"));
//  Serial.print(mpu_pitch);
//  Serial.print('\t');
//  Serial.print(F("roll:"));
//  Serial.print(mpu_roll); 

//  
  Serial.print('\t');
  Serial.print(F("PID_rate: "));
  Serial.print(kp_rate);
  Serial.print('\t');
  Serial.print(ki_rate);
  Serial.print('\t');
  Serial.print(F("kd_rate: "));
  Serial.print(kd_rate,3);
  Serial.print('\t');
  Serial.print(F("PID_stab"));
  Serial.print('\t');
  Serial.print(kp_stab);
  Serial.print('\t');
  Serial.print(ki_stab);
  Serial.print('\t');
  Serial.print(kd_stab,3);
    Serial.print('\t');
    Serial.print("yaw: ");
  Serial.print(kp_rate_yaw,3);
    Serial.print('\t');
  Serial.print(kd_rate_yaw,3);
//  Serial.print('\t');
//  Serial.print("Dial1_2: ");
//  Serial.print(nrf24Data.dial1);
//  Serial.print('\t');
//  Serial.print(nrf24Data.dial2);
//  
  

 
 
  Serial.print('\n');
}
