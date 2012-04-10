/*
   This file is included as part of the CANOPNR distribution. 
   You are free to use this project as you see fit, provided credit is given to all 
   contributors (original and subsequent).

   Copyright 2012  Jose Denrie Enriquez, Alex Bautista, Paloma Field, Sun-il Kim

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.

  ------------------------------------------------------------------------------------------------------------

  Based on original file: MCP2515.h - CAN library -- Written by Frank Kienast in November, 2010
  
  Connections to MCP2515:
  Arduino  MCP2515
  11       MOSI
  12       MISO
  13       SCK
  10       CS 

  //////////////////////////////////////////////////////////////////////////////////////////////
  *****Modified for the CANOPNR. **********
  Modifications include:
  -Modified 1 existing class:
     1. class MCP2515
  //////////////////////////////////////////////////////////////////////////////////////////////

*/

#ifndef MCP2515_h
#define MCP2515_h

#include "Arduino.h." // for Arduino 1.0 , otherwise do #include "WProgram.h"

typedef struct
{
  unsigned short adrsValue;
  boolean isExtendedAdrs;
  unsigned long extendedAdrsValue;
  boolean rtr;
  byte dataLength;
  byte data[8];
}  CANMSG;

class MCP2515
{
  public:
    static boolean initCAN(int baudConst);
	static boolean setCANNormalMode();
	static boolean setCANReceiveonlyMode();
	static boolean setWakeMode();
	static boolean setSleepMode();
	static boolean isAwake();
	static boolean receiveCANMessage(CANMSG *msg, unsigned long timeout);
	static boolean transmitCANMessage(CANMSG msg, unsigned long timeout);
	static boolean getMSG(CANMSG *msg, unsigned long timeout, unsigned long message_addr);
	static boolean CANSNIFF(CANMSG *msg, unsigned short address, unsigned long timeout);
	static boolean SNIFF_ALL(CANMSG *msg);

	/*
	static boolean BRAKE_PRESSURE(CANMSG *msg, unsigned long timeout);
	static boolean ABSCAN(CANMSG *msg, unsigned long timeout);
	static boolean ACCELERATOR(CANMSG *msg, unsigned long timeout);
	*/

	static byte getCANTxErrCnt();
	static byte getCANRxErrCnt();
	static long queryOBD(unsigned char code, char* buffer);
	static byte readReg(byte regno);
	
	private:
	static boolean setCANBaud(int baudConst);
	static void writeReg(byte regno, byte val);
	static void writeRegBit(byte regno, byte bitno, byte val);
//	static byte readReg(byte regno);
};

//Data rate selection constants
#define CAN_BAUD_10K 1
#define CAN_BAUD_50K 2
#define CAN_BAUD_100K 3
#define CAN_BAUD_125K 4
#define CAN_BAUD_250K 5
#define CAN_BAUD_500K 6

#define ABSCAN 0x513
#define ACCELERATOR 0x410
#define BRAKE_PRESSURE 0x511
#define SWAngle 0x511


#define ENGINE_COOLANT_TEMP 0x05
#define ENGINE_RPM          0x0C
#define VEHICLE_SPEED       0x0D
#define MAF_SENSOR          0x10
#define O2_VOLTAGE          0x14
#define THROTTLE		0x11
#define REAL_THROTTLE		0x11
#define FUEL_LEVEL		0x2F
#define RUN_TIME		0x1F
#define INTAKE_TEMP		0x0F

#define PID_REPLY		0x7E8

#endif
