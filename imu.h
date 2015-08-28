#ifndef _IMU_H_
#define _IMU_C_

#include "Arduino.h"

extern float pitch_set, pitch_out, roll_set, roll_out;

extern volatile bool mpuInterrupt;
extern float ypr[3];
extern float euler[3];
extern uint16_t packetSize;                   // estimated packet size  
extern uint16_t fifoCount;  

extern int16_t ax, ay, az;
extern int16_t gx, gy, gz;

void initMPU(void);
void getYPR(void); 
void initMPU_NoDMP(void);
void getMotion6_NoDMP(void);

#endif