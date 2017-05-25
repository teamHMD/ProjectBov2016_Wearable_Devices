/*
 * main_core_0.cpp
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
#include "Acc.h"

#define BAUD_R_DEF 9600
#define BAUD_SOFT_DEF 19200
#define ADJ_DEF 0x28

void setup()
{
  Serial.begin(BAUD_R_DEF);
  delay(50);

  Serial.println("Hello world! I am BOV with HMD.");

  Acc* myacc = new Acc();
  delay(50);

  Minicam* mycam = new Minicam(myacc,pinNumIR,HIGH);
  delay(50);

  height = Serial.parseInt();

  pinMode(pinNumIR, INPUT);
  delay(50);
}

void loop()
{
  if (Get_State() == HIGH)
  {
	  mycam->Set_State(HIGHT);
	  mycam->Play_Cam();

	  u_int8 BTbuff[ONE_BYTE] = {(int)mycam->Get_FPS(), 0, 0, 0, 0, 0, 0, 0};
	  SCBT_WRITE(BT232,&BTbuff,ONE_BYTE);
	  while(!SCBT_SUCCESS(BT232));

  }
}
