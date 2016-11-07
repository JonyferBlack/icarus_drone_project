#include "PID_v1.h"  //modiefied PID-Library working with float only
#include "MPU6050_6Axis_MotionApps20.h"
#include <Servo.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>



//MPU
MPU6050 mpu;                           // mpu interface object
uint8_t mpuIntStatus;                  // mpu statusbyte
uint8_t devStatus;                     // device status    
uint16_t packetSize;                   // estimated packet size  
uint16_t fifoCount;                    // fifo buffer size   
uint8_t fifoBuffer[64];                // fifo buffer 
Quaternion q;                          // quaternion for mpu output
VectorFloat gravity;                   // gravity vector for ypr
float ypr[3] = {0.0f,0.0f,0.0f};       // yaw pitch roll values
volatile bool mpuInterrupt = false;    //interrupt flag
boolean interruptLock = false;



Servo FL, FR, BL, BR;

float sampletime;
int rc_throttle;
int rc_input_limit;
float pwm_FL, pwm_FR, pwm_BL, pwm_BR;
float rc_yaw, rc_pitch, rc_roll;
float mpu_yaw, mpu_pitch, mpu_roll;
float rate_yaw_out, rate_pitch_out, rate_roll_out;
float stab_yaw_out, stab_pitch_out, stab_roll_out;
float kp_rate, ki_rate, kd_rate;
float kp_stab, ki_stab, kd_stab;
float kp_rate_yaw, ki_rate_yaw, kd_rate_yaw;
float kp_stab_yaw, ki_stab_yaw, kd_stab_yaw;
float last_yaw;

float gyro_yaw, gyro_pitch, gyro_roll;
int16_t gx, gy, gz; //Gyro values
//MPU6050 accelgyro;

//Achtung: stab zurzeit als rate zweckentfremdet mpu_pitch <->gyro_pitch..
PID rate_pid_pitch(&gyro_pitch, &rate_pitch_out, &stab_pitch_out, kp_rate, ki_rate, kd_rate, DIRECT); // später hier rc_pitch durch stab_pitch_out ersetzen
PID rate_pid_roll(&gyro_roll, &rate_roll_out, &stab_roll_out, kp_rate, ki_rate, kd_rate, DIRECT); //later replace rc_roll with stab_roll_out
PID stab_pid_pitch(&mpu_pitch, &stab_pitch_out, &rc_pitch, kp_stab, ki_stab, kd_stab, DIRECT); //!!
PID stab_pid_roll(&mpu_roll, &stab_roll_out, &rc_roll, kp_stab, ki_stab, kd_stab, DIRECT); //!!

PID rate_pid_yaw(&gyro_yaw, &rate_yaw_out, &stab_yaw_out, kp_rate_yaw, ki_rate_yaw, kd_rate_yaw, DIRECT);
PID stab_pid_yaw(&mpu_yaw, &stab_yaw_out, &last_yaw, kp_stab_yaw,  ki_stab_yaw, kd_stab_yaw, DIRECT);



float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

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
  stab_yaw_out = 0.0;
  mpu_pitch = 0.0;
  rc_pitch = 0.0;
  stab_pitch_out = 0.0;
  mpu_roll = 0.0;
  rc_roll = 0.0;
  stab_roll_out = 0.0;
  kp_rate = 0.45, ki_rate = 2.2, kd_rate = 0.002;
  kp_stab = 3, ki_stab = 0.0, kd_stab = 0.0;
  kp_rate_yaw = 1.2; ki_rate_yaw = 0.0, kd_rate_yaw = 0.0;
  kp_stab_yaw = 1.2, ki_stab_yaw = 0.0, kd_stab_yaw = 0.0;

  last_yaw = 0.0;
  rate_yaw_out = 0.0;
  rate_pitch_out = 0.0;
  rate_roll_out = 0.0;
  rc_input_limit = 30;

  stab_pid_yaw.SetMode(AUTOMATIC);
  stab_pid_yaw.SetOutputLimits(-500, 500);
  stab_pid_pitch.SetMode(AUTOMATIC);
  stab_pid_pitch.SetOutputLimits(-500, 500);
  stab_pid_roll.SetMode(AUTOMATIC);
  stab_pid_roll.SetOutputLimits(-500, 500);
  rate_pid_yaw.SetMode(AUTOMATIC);
  rate_pid_yaw.SetOutputLimits(-500,500);
  rate_pid_pitch.SetMode(AUTOMATIC);
  rate_pid_pitch.SetOutputLimits(-500,500);
  rate_pid_roll.SetMode(AUTOMATIC);
  rate_pid_roll.SetOutputLimits(-500,500);
  

  sampletime = 2.5;
  stab_pid_yaw.SetSampleTime(sampletime); 
  stab_pid_pitch.SetSampleTime(sampletime);
  stab_pid_roll.SetSampleTime(sampletime);
  rate_pid_yaw.SetSampleTime(sampletime);
  rate_pid_roll.SetSampleTime(sampletime);
  rate_pid_pitch.SetSampleTime(sampletime);
 
}



void loop()
{

  readRX();
  while(!mpuInterrupt && fifoCount < packetSize){
  }

  mpu.getRotation(&gx, &gy, &gz);
  gyro_yaw = -gz/16.4;
  gyro_pitch = -gy/16.4;
  gyro_roll = gx/16.4;
  
  getYPR();  
  mpu_yaw = ypr[0] * 180/M_PI;
  mpu_pitch = ypr[1] * 180/M_PI;
  mpu_roll = ypr[2] * 180/M_PI;
  

  
  if(rc_throttle > 1100)
  {
  stab_pid_yaw.SetTunings(kp_stab_yaw, ki_stab_yaw, kd_stab_yaw);
  stab_pid_pitch.SetTunings(kp_stab, ki_stab, kd_stab);
  stab_pid_roll.SetTunings(kp_stab, ki_stab, kd_stab);
  stab_pid_yaw.Compute();
  stab_pid_pitch.Compute();
  stab_pid_roll.Compute();

  if(abs(rc_yaw) > 5){
    stab_yaw_out = rc_yaw;
    last_yaw = mpu_yaw;
  }
  rate_pid_yaw.SetTunings(kp_rate_yaw, ki_rate_yaw, kd_rate_yaw);
  rate_pid_pitch.SetTunings(kp_rate, ki_rate, kd_rate);
  rate_pid_roll.SetTunings(kp_rate, ki_rate, kd_rate);
  rate_pid_yaw.Compute();
  rate_pid_pitch.Compute();
  rate_pid_roll.Compute();

  pwm_FL = rc_throttle + rate_roll_out + rate_pitch_out - rate_yaw_out; 
  pwm_FR = rc_throttle - rate_roll_out + rate_pitch_out + rate_yaw_out;
  pwm_BL = rc_throttle + rate_roll_out - rate_pitch_out + rate_yaw_out;
  pwm_BR = rc_throttle - rate_roll_out - rate_pitch_out - rate_yaw_out;

  }
  else{

  pwm_FL = rc_throttle;
  pwm_BL = rc_throttle;
  pwm_FR = rc_throttle;
  pwm_BR = rc_throttle;

  last_yaw = mpu_yaw;

  //Reset Integral
  stab_pid_yaw.SetOutputLimits(0.0, 1.0);
  stab_pid_pitch.SetOutputLimits(0.0, 1.0);
  stab_pid_roll.SetOutputLimits(0.0, 1.0);
  rate_pid_yaw.SetOutputLimits(0.0, 1.0);
  rate_pid_pitch.SetOutputLimits(0.0, 1.0);
  rate_pid_roll.SetOutputLimits(0.0, 1.0);

  stab_pid_yaw.SetOutputLimits(-1.0, 0.0);
  stab_pid_pitch.SetOutputLimits(-1.0, 0.0);
  stab_pid_roll.SetOutputLimits(-1.0, 0.0);
  rate_pid_yaw.SetOutputLimits(-1.0, 0.0);
  rate_pid_pitch.SetOutputLimits(-1.0, 0.0);
  rate_pid_roll.SetOutputLimits(-1.0, 0.0);

  stab_pid_yaw.SetOutputLimits(-500, 500);
  stab_pid_pitch.SetOutputLimits(-500, 500);
  stab_pid_roll.SetOutputLimits(-500, 500);
  rate_pid_yaw.SetOutputLimits(-500,500);
  rate_pid_pitch.SetOutputLimits(-500,500);
  rate_pid_roll.SetOutputLimits(-500,500);
  
  }





  
//debug();
 processing();
  
  FL.writeMicroseconds(pwm_FL);
  FR.writeMicroseconds(pwm_FR);
  BL.writeMicroseconds(pwm_BL);
  BR.writeMicroseconds(pwm_BR);
  
}








