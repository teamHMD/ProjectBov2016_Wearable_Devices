/*
 * MiniCam.cpp
 *
 *  Created on: 2016 HMD Project
 *      Team: HMD, Electric Engineering & Electron Engineering, Pusan National Univ.
 *      Authors: Gu-hwan Bae
 *      		   Tae-hyeong Koo
 *      		   Kyeong-Yeop Jeong
 *      		   Hye-ri Bang
 *      		   Jeong-wook Kim
 */

#include <SoftwareSerial.h>
#include <SPI.h>
#include "MiniCam.h"

static int buffp = 0;

Minicam::Minicam(Acc* myacc, int IR_ID, bool state) {
	pixy = new Pixy((XGA_WIDTH,XGA_HEIGHT,ADJ_EFT);
	acc = myacc;

	pinID = IR_ID;
	pixyState = state;

	fps = 0;

	pixy->init();
	delay(50);

	for(int i=0; i < BUFFER_SIZE; i++) {
		block_array[i] = 0;
	}

	distance_x = 0; distance_y = 0;
}

Minicam::~Minicam() {
	pixy = NULL;
	pixyState = LOW;
}

void Minicam::Get_State() {
	return !(pixy->watchdog());
}

void Minicam::Play_Cam() {
	blocks = pixy->getBlocks();

	if (blocks)
	{
		clockRun(CLOCK_BEGIN);
		buffp++;
		if (buffp % 50 == 0)
		{
			sprintf(buf, "Detected %d:\n", blocks);
			Serial.print(buf);

			for (int i = 0; i < blocks; i++)
			{
				sprintf(buf, "  block %d: ", i);
				Serial.print(buf);
				pixy->blocks[i].print();

				// c_x is x centroid value of principle point;
				// c_y is y centroid value of principle point;

				Get_Real_Distance(block_array, j, pixy.blocks[j].x, pixy.blocks[j].y, pixy.blocks[j].width, pixy.blocks[j].height);//To return the real world's distance, Send values to this function. one_diff is pixel meter.

				Serial.print(" x = "); Serial.print(distance_x);
				Serial.print(" y = "); Serial.println(distance_y);
			}
		}

		u_int8 BTbuff[ONE_BYTE] = {distance_x, distance_y, 0, 0, 0, 0, 0, 0};
		SCBT_WRITE(BT232,&BTbuff,ONE_BYTE);
		while(!SCBT_SUCCESS(BT232));

		clockRun(CLOCK_END);
	}
}

void Minicam::Get_Real_Distance(double** block_calculated_array, int index, int x, int y, int w, int h) {

	double u = 0, v = 0;
	double u_diff = 0, v_diff = 0; // u and v are real distance of object's location. u_diff and v_diff are object's width and height.
	int x_diff = 0, y_diff = 0, r_diff = 0;

	double theta = 0, distance = 0;

	while (theta < 0 && theta >= 90) {
		theta = Get_Pitch();
		Serial.print("theta = ");
		Serial.print(theta);
		if (theta < 0 && theta >= 90) {
			Serial.println(" Received angle data was improper! ");
		}
	}

	//distance = void Get_Sensor_Distance(theta);
	distance = Get_Sensor_Distance(theta);

	y = y + (h * 0.5);
	x = x + (w * 0.5);

	if (y <= c_y)//upper
	{
		y_diff = abs(c_y - y);
		u = (distance * y_diff) / (f_y * sin(theta) - y_diff * cos(theta));
		block_calculated_array[index][1] = (distance * cos(theta)) + u;

		x_diff = abs(c_x - x);

		r_diff = sqrt(sq(x_diff) + sq(y_diff));

		v = abs(	sqrt(	((sq(r_diff) * sq(distance + u * cos(theta))) / sq(f_x)) - sq(u * sin(theta))));

		if (x < c_x) //left
		{
			v = -v;
		}

		block_calculated_array[index][0] = v;

	}
	if (y > c_y) //under
	{
		y_diff = abs(y - c_y);
		u = (distance * y_diff) / (f_y * sin(theta) + y_diff * cos(theta));
		block_calculated_array[index][1] = (distance * cos(theta)) - u;

		x_diff = abs(c_x - x);

		r_diff = sqrt(sq(x_diff) + sq(y_diff));

		v = abs( sqrt(((sq(r_diff) * sq(distance - u * cos(theta))) / sq(f_x)) - sq(u * sin(theta))));

		if (x < c_x) //left
		{
			v = -v;
		}

		block_calculated_array[index][0] = v;
	}
}

double Minicam::Get_Pitch() {
	acc->pitch(RADIAN);
}

double Minicam::Get_Sensor_Distance(double theta) {
	return Get_ir_distance();
}

void Minicam::Set_State(bool newState) {
	delay_us(200);
	pixyState = newState;
	delay_us(200);
}

double Minicam::Get_X() {
	return distance_x;
}

double Minicam::Get_Y() {
	return distance_y;
}

float Minicam::Get_FPS() {
	return fps;
}

void Minicam::clockRun(int argu) {
	if(argu == CLOCK_BEGIN) {
		t = clock();
	}
	else if(argu == CLOCK_END) {
		t = clock() - t;
		fps = (((float)t)/CLOCKS_PER_SEC);
	}
}

double Minicam::Get_Sensor_Distance(double theta)
{
  int i = 0; const int n = 10;

  float distance = 0;
  float ar[10];
  float sum = 0;
  float average = 0;

  while (i < 10)
  {

    distance = Get_ir_distance();
    delay(50);
    if ((height / sin(theta)) - 95 <= distance &&  distance <= (height / sin(theta)) + 200)  //d의 범위 1차필터
    {
      Serial.print("Range = ");    Serial.print(distance);   Serial.println("cm");

      ar[i] = distance;
      i++;
    }
  }
  Sorting_ir_sensor_data(ar, n);
  Serial.println(" Re_Arrangement in Ascending Order !");
  for (i = 0; i < n; i++)
  {
    Serial.print(ar[i]); Serial.println("cm");
  }
  for (i = 2; i < 8; i++)              //최소값 최대값 2개씩 제외하고 평균
  {
    sum += ar[i];
  }
  average = sum / 6;
  Serial.print("Average : "); Serial.println(average);
  i = 0;
}

float Minicam::Get_ir_distance()
{

  int data = analogRead(pinNumIR);
  Serial.println(data);

  float distance_cm = (1 / (0.0000335 * data - 0.007085)); //float volt = map(data, 0, 1023, 0, 5000);
  delay(30);
  return distance_cm;

}
