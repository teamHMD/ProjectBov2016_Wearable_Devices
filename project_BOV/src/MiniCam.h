/*
 * MiniCam.h
 *
 *  Created on: 2016 HMD Project
 *      Team: HMD, Electric Engineering & Electron Engineering, Pusan National Univ.
 *      Authors: Gu-hwan Bae
 *      		   Tae-hyeong Koo
 *      		   Kyeong-Yeop Jeong
 *      		   Hye-ri Bang
 *      		   Jeong-wook Kim
 */

#include <Pixy.h>
#include <ADXL345.h>
#include "Acc.h"
#include <time.h>
#include <math.h>
#include "/btn/include"

#ifndef MINICAM_H_
#define MINICAM_H_

#ifndef NULL
#define NULL 0
#endif

#ifndef XGA_WIDTH
#define XGA_WIDTH 320
#endif

#ifndef XGA_HEIGHT
#define XGA_HEIGHT 240
#endif

#ifndef PIXY_PARAMETERS
#define CX 160	//  c_x and c_y are centroid value.
#define CY 120	//  Pp(c_x, c_y) <- Pp i s point of principle point.
#define DUMMY_PIXEL 20
#define FX 187.9	//  f_x and f_y are focal length.
#define FY 182.1
#define THETA_ERROR_COMP 6.7
#endif

#ifndef BUFFER_SIZE
#define BUFFER_SIZE 32
#endif

#ifndef CLUSTER_PARAMETERS
#define CLUSTER_HEIGHT 7
#define CLUSTER_WIDTH 4
#endif

#define BAUD_R_DEF 9600
#define BAUD_SOFT_DEF 19200
#define ADJ_DEF 0x28

extern clock_t t;

class Minicam{
private :
	Pixy* pixy;
	Acc* acc;

	int pinID;
	bool pixyState;

	char buf[BUFFER_SIZE];
	uint16_t blocks;

	double block_array[CLUSTER_HEIGHT][CLUSTER_WIDTH];
	double distance_x;
	double distance_y;

	float fps;

public :
	Minicam(Acc*, int, bool);
	~Minicam();
	void Play_Cam();

	void Get_Real_Distance(double**, int, int, int, int, int);

	double Get_Pitch();
	double Get_Sensor_Distance(double);

	void Set_State(bool);

	double Get_X();
	double Get_Y();

	float Get_FPS();

	double Get_Sensor_Distance(double theta);
	float Get_ir_distance();
};

#endif /* MINICAM_H_ */
