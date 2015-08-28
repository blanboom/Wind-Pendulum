/*****************************************************************
 * 公式：
 *		theta = atan(sqrt(pow(tan(ypr[1]), 2.0) + pow(tan(ypr[2]), 2.0))) * 180/M_PI
 *		phi   = acos(tan(ypr[2]) / (sqrt(pow(tan(ypr[1]), 2.0) + pow(tan(ypr[2]), 2.0)))) * 180/M_PI  (需判断正负，进行修正)
 *		roll? = atan(tan(theta) * cos(phi))
 *		pitch?= atan(tan(theta) * sin(phi))
 *****************************************************************/

#include "IMU.h"
#include "PID_v1.h"
#include "interface.h"

uint8_t mode = 1;

#define _DEBUG


void setup() {
	initMotor();
	interface_init();
	Serial.begin(115200);
	Serial.println("Welcome!");

	mode = interface_input_mode();
	switch(mode) {
	case 1:
		app_mode1();
		break;
	case 2:
		app_mode2();
		break;
	case 3:
		app_mode3();
		break;
	case 4:
		app_mode4();
		break;
	case 5:
		app_mode5();
		break;
	default:
		break;
	}
}

void loop(){

	// /* ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓ 读取角度 ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓ */
	// while (!mpuInterrupt && fifoCount < packetSize) {
	// 	/* Do nothing while MPU is not working
	// 	 * This should be a VERY short period
	// 	 */
	// }
	// getYPR();
	// /* ↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑ 50ms ↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑ */

	// computePID();
	// updateMotor();
	// delay(100);
}

void app_mode1(void) {
	initMPU_NoDMP();
	//initRegulators();
	Serial.println("App mode1 initialized!");

	beep();
	for(;;) {
		getMotion6_NoDMP();
		if(gx > 0) {
			motorOutput13(255);
			//motorOutput24(255);
		} else {
			motorOutput13(-255);
			//motorOutput24(-255);
		}

		delay(50);
	}
}

void app_mode2(void) {
	float theta_in, theta_out, theta_set, theta_last = 0;
	PID theta_pid(&theta_in, &theta_out, &theta_set, 5.0, 1.3, 2.0, DIRECT);
	int16_t gx_last = 0;
	bool a = 0;

	// motorOutput13(100);
	// delay(100);
	// motorOutput13(0);

	theta_pid.SetMode(AUTOMATIC);
	theta_pid.SetOutputLimits(-255, 255);
	theta_pid.SetSampleTime(1500);

	
	Serial.println("App mode2 initialized!");
	theta_set = interface_input_length();
	
	if(theta_set <= 35) { theta_pid.SetTunings(1.0, 0.7, 1.0); }
	else if(theta_set <= 45) { theta_pid.SetTunings(8.0, 7.0, 2.0); }
	else { theta_pid.SetTunings(5.0, 7.0, 2.0); }
	Serial.print("Length set: ");
	Serial.print(theta_set);
	Serial.print(". ");
	theta_set = atan((theta_set / 2) / 88.7);
	theta_set *= 180 /M_PI;
	Serial.print("Angle set: ");
	Serial.print(theta_set);
	Serial.print(". \n");

	initMPU();

	beep();
	for(;;) {
		if (gx == 0 || (gx > 0 && gx_last < 0) || (gx < 0 && gx_last > 0)) {   // TODO: 如果速度不够快，可以提高 DMP 频率；或 gx 绝对值落在一定范围内时就直接更新 theta
			if(a == 0) {
				a = 1;
				theta_in = atan(sqrt(pow(tan(ypr[1]), 2.0) + pow(tan(ypr[2]), 2.0))) * 180.0 / M_PI;
			} else {
				a = 0;
				theta_in += atan(sqrt(pow(tan(ypr[1]), 2.0) + pow(tan(ypr[2]), 2.0))) * 180.0 / M_PI;
				theta_in /= 2.0;
				if(theta_in > 80) {
					theta_in = theta_last;
					theta_pid.Compute();
				} else {
					theta_pid.Compute();
				}
				theta_last = theta_in;
			}
		}

		Serial.println(theta_in);
		Serial.println(", ");
		Serial.println(theta_out);
		
		if(abs(theta_out) < 2) { theta_out = 100; }
		if(gx > 0) {
			motorOutput13(theta_out);
		} else {
			motorOutput13(-theta_out);
		}

		gx_last = gx;

		while (!mpuInterrupt && fifoCount < packetSize) {}
		getYPR();
		getMotion6_NoDMP();
	 }
}


void app_mode3(void) {
	uint16_t angle_input = 0;
	int16_t motor_val1, motor_val2;

	initMPU_NoDMP();
	//initRegulators();
	Serial.println("App mode3 initialized!");
	Serial.println("Input radius:");
	angle_input = interface_input_angle();

	if(angle_input == 120) { angle_input = 130; }

	// switch(angle_input) {
	// 	case 0:
	// 		angle_input = 180;
	// 		break;
	// 	case 10:
	// 		angle_input = 0;
	// 		break;
	// 	case 20:
	// 		angle_input = 0;
	// 		break;
	// 	case 30:
	// 		angle_input = 10;
	// 		break;
	// 	case 40:
	// 		angle_input = 15;
	// 		break;
	// 	case 50:
	// 		angle_input = 20;
	// 		break;
	// 	case 60:
	// 		angle_input = 30;
	// 		break;
	// 	case 70:
	// 		angle_input = 40;
	// 		break;
	// 	case 80:
	// 		angle_input = 50;
	// 		break;
	// 	case 90:
	// 		angle_input = 90;
	// 		break;
	// 	case 100:
	// 		angle_input = 120;
	// 		break;
	// 	case 110:
	// 		angle_input = 130;
	// 		break;
	// 	case 120:
	// 		angle_input = 150;
	// 		break;
	// 	case 130:
	// 		angle_input = 152;
	// 		break;
	// 	case 140:
	// 		angle_input = 154;
	// 		break;
	// 	case 150:
	// 		angle_input = 156;
	// 		break;
	// 	case 160:
	// 		angle_input = 158;
	// 		break;
	// 	case 170:
	// 		angle_input = 160;
	// 		break;
	// 	case 180:
	// 		angle_input = 180;
	// 		break;
	// 	default:
	// 		angle_input = 90;
	// 		break;
	// }

	motor_val1 = 120 * sin(angle_input * M_PI / 180);
	motor_val2 = 120 * cos(angle_input * M_PI / 180);

	beep();
	for(;;) {
		getMotion6_NoDMP();
		// if(angle_input <= 90) {
			if (gx > 0) {
				motorOutput13(motor_val1);
				motorOutput24(motor_val2);
			} else {
				motorOutput13(-motor_val1);
				motorOutput24(-motor_val2);
			}			
		// } else {
		// 	if (gx > 0) {
		// 		motorOutput13(motor_val1);
		// 		motorOutput24(-motor_val2);
		// 	} else {
		// 		motorOutput13(-motor_val1);
		// 		motorOutput24(motor_val2);
		// 	}	
		// }

	}
}

void app_mode4(void) {
	float kp_x = -0.1, kp_y = -0.1;
	int16_t output_x = 0, output_y = 0;
	initMPU_NoDMP();
	delay(400);
	//initRegulators();
	Serial.println("App mode4 initialized!");

	beep();
	for(;;) {
		getMotion6_NoDMP();

		output_x = gx * kp_x;
		output_y = gy * kp_y;

		//Serial.println(output_x);

		if(output_x < -255) { output_x = -255; }
		else if(output_x > 255 ) { output_x =  255; }
		if(output_y < -255) { output_y = -255; }
		else if(output_y > 255 ) { output_y =  255; }

		motorOutput13(output_x);
		motorOutput24(output_y);

		delay(4);
	}
}

void app_mode5(void) {
	const uint8_t delayTime = 120; // 每步时间
	float currentAngle = 0.0;
	float i;

	unsigned long lastTime = 0, currentTime = 0; 
	float theta_in,   theta_out,   theta_set,   theta_last   = 0;   // PID 输入输出值
	float theta_in_2, theta_out_2, theta_set_2, theta_last_2 = 0;   // PID 输入输出值（另一个方向）
	PID theta_pid(  &theta_in, &theta_out, &theta_set, 6.0, 1.2, 5.1, DIRECT);
	PID theta_pid_2(&theta_in_2, &theta_out_2, &theta_set_2, 8.0, 1.5, 3.5, DIRECT);  // 另一个方向的 PID 控制器
	int16_t gx_last = 0, gy_last = 0;
	bool a = 0, a_2 = 0;

	theta_pid.SetMode(AUTOMATIC);
	theta_pid.SetOutputLimits(-255, 255);
	theta_pid.SetSampleTime(1500);
	theta_pid_2.SetMode(AUTOMATIC);
	theta_pid_2.SetOutputLimits(-255, 255);
	theta_pid_2.SetSampleTime(1500);

	
	Serial.println("App mode5 initialized!");
	theta_set = interface_input_radius() * 2.0 + 1.0;
	theta_set_2 = theta_set;

	initMPU();

	Serial.print("Length set: ");
	Serial.print(theta_set);
	Serial.print(". ");
	theta_set    = atan((theta_set   / 2) / 88.7);
	theta_set   *= 180 /M_PI;
	theta_set_2  = atan((theta_set_2 / 2) / 88.7);
	theta_set_2 *= 180 /M_PI;
	Serial.print("Angle set: ");
	Serial.print(theta_set);
	Serial.print(". \n");


	
	/************************** 起摆 **************************/
	// currentTime = millis();
	// for(;;) {
	// 	for (float currentAngle = 0.0; currentAngle < 360.0; currentAngle += 30.0) {
	// 		motorOutput13(80.0 * sin(currentAngle * M_PI / 180.0));
	// 		motorOutput24(80.0 * cos(currentAngle * M_PI / 180.0));
	// 		delay(delayTime);
	// 	}
	// 	if(millis() - currentTime > 7000) { break; }
	// }

	/************************** 维持 **************************/
	beep();
	for(;;) {
		while (!mpuInterrupt && fifoCount < packetSize) {}
		getYPR();
		getMotion6_NoDMP();

	  	if (gx == 0 || (gx > 0 && gx_last < 0) || (gx < 0 && gx_last > 0)) {
			if(a == 0) {
				a = 1;
				theta_in = atan(sqrt(pow(tan(ypr[1]), 2.0) + pow(tan(ypr[2]), 2.0))) * 180.0 / M_PI;
			} else {
				a = 0;
				theta_in += atan(sqrt(pow(tan(ypr[1]), 2.0) + pow(tan(ypr[2]), 2.0))) * 180.0 / M_PI;
				theta_in /= 2.0;
				if(theta_in > 80) {
					theta_in = theta_last;
					theta_pid.Compute();
				} else {
					theta_pid.Compute();
				}
				theta_last = theta_in;
			}
		}
		if (gy == 0 || (gy > 0 && gy_last < 0) || (gy < 0 && gy_last > 0)) {
			if(a_2 == 0) {
				a_2 = 1;
				theta_in_2 = atan(sqrt(pow(tan(ypr[1]), 2.0) + pow(tan(ypr[2]), 2.0))) * 180.0 / M_PI;
			} else {
				a_2 = 0;
				theta_in_2 += atan(sqrt(pow(tan(ypr[1]), 2.0) + pow(tan(ypr[2]), 2.0))) * 180.0 / M_PI;
				theta_in_2 /= 2.0;
				if(theta_in_2 > 80) {
					theta_in_2 = theta_last_2;
					theta_pid_2.Compute();
				} else {
					theta_pid_2.Compute();
				}
				theta_last_2 = theta_in_2;
			}
		}

		if(abs(theta_out) < 2) { theta_out = 60.0; }
		if(abs(theta_out_2) < 2) { theta_out_2 = 60.0; }

		currentTime = millis();
		i = (currentTime - lastTime) / delayTime;
		if(i > 0){
			currentAngle += 30.0 * i;
			if(currentAngle >= 360) { currentAngle = 0.0; }
			motorOutput13(theta_out   * sin(currentAngle * M_PI / 180.0));
			motorOutput24(theta_out_2 * cos(currentAngle * M_PI / 180.0));
			lastTime = currentTime;
		}

		gx_last = gx;
		gy_last = gy;
	}
}

void app_mode5_backup2(void) {
	float theta_in,   theta_out,   theta_set,   theta_last   = 0;   // PID 输入输出值
	float theta_in_2, theta_out_2, theta_set_2, theta_last_2 = 0;   // PID 输入输出值（另一个方向）
	PID theta_pid(  &theta_in, &theta_out, &theta_set, 1.0, 0.0001, 0.0001, DIRECT);
	PID theta_pid_2(&theta_in_2, &theta_out_2, &theta_set_2, 1.0, 0.0001, 0.0001, DIRECT);  // 另一个方向的 PID 控制器
	int16_t gx_last = 0, gy_last = 0;
	bool a = 0, a_2 = 0;
	unsigned long startTime = 0;


	theta_pid.SetMode(AUTOMATIC);
	theta_pid.SetOutputLimits(-255, 255);
	theta_pid.SetSampleTime(1500);
	theta_pid_2.SetMode(AUTOMATIC);
	theta_pid_2.SetOutputLimits(-255, 255);
	theta_pid_2.SetSampleTime(1500);

	
	Serial.println("App mode5 initialized!");
	theta_set = interface_input_radius() * 2;
	theta_set_2 = theta_set;

	initMPU();

	Serial.print("Length set: ");
	Serial.print(theta_set);
	Serial.print(". ");
	theta_set    = atan((theta_set   / 2) / 88.7);
	theta_set   *= 180 /M_PI;
	theta_set_2  = atan((theta_set_2 / 2) / 88.7);
	theta_set_2 *= 180 /M_PI;
	Serial.print("Angle set: ");
	Serial.print(theta_set);
	Serial.print(". \n");

	/**************************** 起摆 ****************************/
	startTime = millis();
	for(;;) {
		while (!mpuInterrupt && fifoCount < packetSize) {}
		getYPR();
		getMotion6_NoDMP();

		Serial.println(gx);
		if (gx == 0 || (gx > 0 && gx_last < 0) || (gx < 0 && gx_last > 0)) {   // TODO: 如果速度不够快，可以提高 DMP 频率；或 gx 绝对值落在一定范围内时就直接更新 theta
			if(a == 0) {
				a = 1;
				theta_in = atan(sqrt(pow(tan(ypr[1]), 2.0) + pow(tan(ypr[2]), 2.0))) * 180.0 / M_PI;
			} else {
				a = 0;
				theta_in += atan(sqrt(pow(tan(ypr[1]), 2.0) + pow(tan(ypr[2]), 2.0))) * 180.0 / M_PI;
				theta_in /= 2.0;
				if(theta_in > 80) {
					Serial.print("theta_1: ");
					Serial.println(theta_in);
					theta_in = theta_last;
					theta_pid.Compute();
				} else {
					Serial.print("theta_2: ");
					Serial.println(theta_in);
					theta_pid.Compute();
				}
				theta_last = theta_in;
			}
			if(millis() - startTime >= 20000) { break; }
		}
		
		if(abs(theta_out) < 3) { theta_out = 120; }
		if(gx > 0) {
			motorOutput13(theta_out);
		} else {
			motorOutput13(-theta_out);
		}

		gx_last = gx;
	 }

	 /**************************** 给一个 phi 方向的力 ****************************/
	 motorOutput24(255); 
	 delay(750); 
	 motorOutput24(-255); 
	 delay(750); 
	 motorOutput24(0); 

	 /***************************** 通过 PID 保持稳定 *****************************/
	 /** 思路：
	  * 在 roll 和 pitch 方向测出两组最大的 theta, 分别进行 PID
	  */
	  //theta_pid.SetTunings(20.0, 19.0, 10.0);
	  theta_pid_2.SetTunings(20.0, 5.0, 10.0);
	  for(;;) {
	  	while (!mpuInterrupt && fifoCount < packetSize) {}
		getYPR();
		getMotion6_NoDMP();

		if(abs(theta_out) < 3) { theta_out = 120; }
		if(gx > 0) {
			motorOutput13(theta_out);
		} else {
			motorOutput13(-theta_out);
		}
		if(abs(theta_out_2) < 3) { theta_out_2 = 120; }

	  	if (gx == 0 || (gx > 0 && gx_last < 0) || (gx < 0 && gx_last > 0)) {
	  		if(ypr[2] > 0) {
				motorOutput24(theta_out_2);
			} else {
				motorOutput24(-theta_out_2);
			}

			if(a == 0) {
				a = 1;
				theta_in = atan(sqrt(pow(tan(ypr[1]), 2.0) + pow(tan(ypr[2]), 2.0))) * 180.0 / M_PI;
			} else {
				a = 0;
				theta_in += atan(sqrt(pow(tan(ypr[1]), 2.0) + pow(tan(ypr[2]), 2.0))) * 180.0 / M_PI;
				theta_in /= 2.0;
				if(theta_in > 80) {
					theta_in = theta_last;
					theta_pid.Compute();
				} else {
					theta_pid.Compute();
				}
				theta_last = theta_in;
			}
		}
		if (gy == 0 || (gy > 0 && gy_last < 0) || (gy < 0 && gy_last > 0)) {
			if(a_2 == 0) {
				a_2 = 1;
				theta_in_2 = atan(sqrt(pow(tan(ypr[1]), 2.0) + pow(tan(ypr[2]), 2.0))) * 180.0 / M_PI;
			} else {
				a_2 = 0;
				theta_in_2 += atan(sqrt(pow(tan(ypr[1]), 2.0) + pow(tan(ypr[2]), 2.0))) * 180.0 / M_PI;
				theta_in_2 /= 2.0;
				if(theta_in_2 > 80) {
					theta_in_2 = theta_last_2;
					theta_pid_2.Compute();
				} else {
					theta_pid_2.Compute();
				}
				theta_last_2 = theta_in_2;
			}
		}

		gx_last = gx;
		gy_last = gy;
	  }
}

void app_mode5_backup(void) {
	float pitch_out, pitch_set, roll_out, roll_set;
	PID pitchReg(&ypr[1], &pitch_out, &pitch_set, 15.0, 0.0, 0.0, DIRECT);
	PID rollReg(&ypr[2],  &roll_out, &roll_set, 15.0, 0.0, 0.0, DIRECT);
	float phi, theta  = 0;
	float roll, pitch = 0;
	unsigned long time1 = 0, time2 = 0, time_tmp;

	phi = 11.16;  // degree

	pitchReg.SetMode(AUTOMATIC);
	pitchReg.SetOutputLimits(-255, 255);
	pitchReg.SetSampleTime(20);
	rollReg.SetMode(AUTOMATIC);
	rollReg.SetOutputLimits(-255, 255);
	rollReg.SetSampleTime(20);

	initMPU();

	for(;;) {
		while (!mpuInterrupt && fifoCount < packetSize) {}
		getYPR();

		time_tmp = millis();
		if(time_tmp - time1 > 20) {
			time1 = time_tmp;
			ypr[1] = ypr[1] * 180 / M_PI;
			ypr[2] = ypr[2] * 180 / M_PI;
			roll_set  = atan(tan(theta) * cos(phi)) * 180 / M_PI;
			pitch_set = atan(tan(theta) * sin(phi)) * 180 / M_PI;
			pitchReg.Compute();
			rollReg.Compute();
			motorOutput13(roll_out);          //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
			motorOutput24(pitch_out);         // NEW WAY：  把此处的 roll_out 和 pitch_out 对调，应该可以做圆周运动
											  // NEW WAY 2:	在基础要求 2 的基础上，当摆摆到指定点，给一个垂直方向的力	
		}

		// 每步 15 度，共 24 步. 在 30s 转 3 圈，每步 416.67ms
		// time_tmp = millis();
		// if(time_tmp - time2 > 400) {
		// 	time2 = time_tmp;	
		// 	theta += 15;
		// 	if(theta >= 360) { theta = 0; }		
		// }
	}
}

void initMotor(void) {
	pinMode(5, OUTPUT);
	pinMode(6, OUTPUT);
	pinMode(7, OUTPUT);
	pinMode(8, OUTPUT);

	//31.25kHz
	TCCR3B = TCCR3B & 0b11111000 | 0x01;
	TCCR4B = TCCR4B & 0b11111000 | 0x01;
}

inline void motorOutput1(uint8_t speed) {
	analogWrite(5, speed);
}

inline void motorOutput2(uint8_t speed) {
	analogWrite(6, speed);
}

inline void motorOutput3(uint8_t speed) {
	analogWrite(7, speed);
}

inline void motorOutput4(uint8_t speed) {
	analogWrite(8, speed);
}

inline void motorOutput13(int16_t speed) {
	if(speed < 0) {
		motorOutput1(abs(speed));
		motorOutput3(0);
	} else {
		motorOutput1(0);
		motorOutput3(speed);
	}
}

inline void motorOutput24(int16_t speed) {
	if(speed < 0) {
		motorOutput2(abs(speed));
		motorOutput4(0);
	} else {
		motorOutput2(0);
		motorOutput4(speed);
	}
}

inline void beep(void) {
	// pinMode(12, OUTPUT);
	// digitalWrite(12, LOW);
	// delay(250);
	// pinMode(12, INPUT);
}
