/*
 * Acc.cpp
 *
 *  Created on: 2016 HMD Project
 *      Team: HMD, Electric Engineering & Electron Engineering, Pusan National Univ.
 *      Authors: Gu-hwan Bae
 *      		   Tae-hyeong Koo
 *      		   Kyeong-Yeop Jeong
 *      		   Hye-ri Bang
 *      		   Jeong-wook Kim
 */

#include "Acc.h"

Acc::Acc() {
	acc_x = 0;
	acc_y = 0;
	acc_z = 0;
	rawX = 90;
	rawY = 90;
	rawZ = 90;
	acc_X = 180;
	acc_Y = 180;
	acc_Z = 180;
}

double Acc::pitch(int mode) {
  adxl.readAccel(&acc_x, &acc_y, &acc_z); //read the accelerometer values and store them in variables  x,y,z

  double pitch_rad = 0;

  rawX = acc_x - 7;
  rawY = acc_y - 6;
  rawZ = acc_z + 10;

  acc_X = rawX / 256.00; // used for angle calculations
  acc_Y = rawY / 256.00; // used for angle calculations
  acc_Z = rawZ / 256.00; // used for angle calculations

  rolldeg = 90 - 180 * (atan(acc_Y / sqrt(acc_X * acc_X + acc_Z * acc_Z))) / PI; // calculated angle in degrees
  pitchdeg = 90 - 180 * (atan(acc_X / sqrt(acc_Y * acc_Y + acc_Z * acc_Z))) / PI; // calculated angle in degrees
  // print out values:

  rolldeg = rolldeg + theta_error;
  pitch_rad  =  (rolldeg * PI)  / 180;
  //Serial.println( pitch_rad );
  //pitch <-> roll , Shifting both

  //Serial.print(" pitch (deg) = "); Serial.print(rolldeg);      // calculated angle in degrees
  //Serial.print(" roll (deg) = "); Serial.print(pitchdeg);
  //Serial.print(" pitch (rad) = "); Serial.println(pitch_rad);

  if (mode == RADIAN)
	  return pitch_rad;
  else
	  return rolldeg;
}
