/*
 * Acc.h
 *
 *  Created on: 2016 HMD Project
 *      Team: HMD, Electric Engineering & Electron Engineering, Pusan National Univ.
 *      Authors: Gu-hwan Bae
 *      		   Tae-hyeong Koo
 *      		   Kyeong-Yeop Jeong
 *      		   Hye-ri Bang
 *      		   Jeong-wook Kim
 */

#ifndef ACC_H_
#define ACC_H_

#include <ADXL345.h>
#include <math.h>

#ifndef PI
#define PI 3.141592
#endif

#ifndef RADIAN
#define RADIAN 1
#endif

class Acc {
private :
	// Values for Accelerometer
	int acc_x, acc_y, acc_z;
	int rawX, rawY, rawZ;
	float acc_X, acc_Y, acc_Z;
	//float rollrad, pitchrad;
	double rolldeg, pitchdeg;
public :
	Acc();
	double pitch(int);
};

#endif /* ACC_H_ */
