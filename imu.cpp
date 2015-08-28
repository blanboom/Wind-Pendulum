#include "IMU.h"
#include <Wire.h>
#include <I2Cdev.h>
#include <helper_3dmath.h>
#include <MPU6050_6Axis_MotionApps20.h>

float pitch_set, pitch_out, roll_set, roll_out;



inline void dmpDataReady();


/*  MPU variables
 *
 */

MPU6050 mpu;                           // mpu interface object

uint8_t mpuIntStatus;                  // mpu statusbyte
uint8_t devStatus;                     // device status    
uint16_t packetSize;                   // estimated packet size  
uint16_t fifoCount;                    // fifo buffer size   
uint8_t fifoBuffer[64];                // fifo buffer 

Quaternion q;                          // quaternion for mpu output
VectorFloat gravity;                   // gravity vector for ypr
float ypr[3] = {0.0f,0.0f,0.0f};       // yaw pitch roll values
float euler[3];         // [psi, theta, phi]    Euler angle container


int16_t ax, ay, az;
int16_t gx, gy, gz;

volatile bool mpuInterrupt = false;    //interrupt flag

void initMPU(){
  
  Wire.begin();
  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  if(devStatus == 0){
  
    mpu.setDMPEnabled(true);
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    packetSize = mpu.dmpGetFIFOPacketSize();
    
  }
}

void initMPU_NoDMP(void){
  
  Wire.begin();
  mpu.initialize();
}

void getMotion6_NoDMP(void) {
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
}

/*  getYPR function
 *
 *  gets data from MPU and
 *  computes pitch, roll, yaw on the MPU's DMP
 */
void getYPR(){
  
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    
    if((mpuIntStatus & 0x10) || fifoCount >= 1024){ 
      
      mpu.resetFIFO(); 
    
    }else if(mpuIntStatus & 0x02){
    
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
  
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      
      fifoCount -= packetSize;
    
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      //mpu.dmpGetEuler(euler, &q);
    }
}

inline void dmpDataReady() {
    mpuInterrupt = true;
}