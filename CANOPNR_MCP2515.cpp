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

  Based on original file: MCP2515.cpp - CAN library --  Written by Frank Kienast in November, 2010
  
  Connections to MCP2515:
  Arduino  MCP2515
  11       MOSI
  12       MISO
  13       SCK
  10       CS 


  //////////////////////////////////////////////////////////////////////////////////////////////
  *****Modified for the CANOPNR. **********
  Modifications include:
  -Added 7 new methods:
     1. boolean MCP2515::setSleepMode()
     2. boolean MCP2515::isAwake()
     3. boolean MCP2515::getMSG(CANMSG *msg, unsigned long timeout, unsigned long message_addr)
     4. boolean MCP2515::SNIFF_ALL(CANMSG *msg)
     5. boolean MCP2515::CANSNIFF(CANMSG *msg, unsigned short address, unsigned long timeout)
     6. boolean MCP2515::receiveCANMessage(CANMSG *msg, unsigned long timeout)
     7. long MCP2515::queryOBD(unsigned char pid, char *buffer)
  -Modified 1 existing method:
     1. boolean MCP2515::setCANNormalMode()
  //////////////////////////////////////////////////////////////////////////////////////////////
*/


#include "Arduino.h"
#include "SPI.h"
#include "CANOPNR_MCP2515.h"

#define SLAVESELECT 10

//MCP2515 Registers
#define RXF0SIDH 0x00
#define RXF0SIDL 0x01
#define RXF0EID8 0x02
#define RXF0EID0 0x03
#define RXF1SIDH 0x04
#define RXF1SIDL 0x05
#define RXF1EID8 0x06
#define RXF1EID0 0x07
#define RXF2SIDH 0x08
#define RXF2SIDL 0x09
#define RXF2EID8 0x0A
#define RXF2EID0 0x0B
#define BFPCTRL 0x0C
#define TXRTSCTRL 0x0D
#define CANSTAT 0x0E
#define CANCTRL 0x0F
#define RXF3SIDH 0x10
#define RXF3SIDL 0x11
#define RXF3EID8 0x12
#define RXF3EID0 0x13
#define RXF4SIDH 0x14
#define RXF4SIDL 0x15
#define RXF4EID8 0x16
#define RXF4EID0 0x17
#define RXF5SIDH 0x18
#define RXF5SIDL 0x19
#define RXF5EID8 0x1A
#define RXF5EID0 0x1B
#define TEC 0x1C
#define REC 0x1D
#define RXM0SIDH 0x20
#define RXM0SIDL 0x21
#define RXM0EID8 0x22
#define RXM0EID0 0x23
#define RXM1SIDH 0x24
#define RXM1SIDL 0x25
#define RXM1EID8 0x26
#define RXM1EID0 0x27
#define CNF3 0x28
#define CNF2 0x29
#define CNF1 0x2A
#define CANINTE 0x2B
	#define MERRE 7
	#define WAKIE 6
	#define ERRIE 5
	#define TX2IE 4
	#define TX1IE 3
	#define TX0IE 2
	#define RX1IE 1
	#define RX0IE 0
#define CANINTF 0x2C
	#define MERRF 7
	#define WAKIF 6
	#define ERRIF 5
	#define TX2IF 4
	#define TX1IF 3
	#define TX0IF 2
	#define RX1IF 1
	#define RX0IF 0
#define EFLG 0x2D
#define TXB0CTRL 0x30
	#define TXREQ 3
#define TXB0SIDH 0x31
#define TXB0SIDL 0x32
	#define EXIDE 3
#define TXB0EID8 0x33
#define TXB0EID0 0x34
#define TXB0DLC 0x35
  #define TXRTR 7
#define TXB0D0 0x36 

#define RXB0CTRL 0x60
	#define RXM1 6
	#define RXM0 5
	#define RXRTR 3
	// Bits 2:0 FILHIT2:0
#define RXB0SIDH 0x61
#define RXB0SIDL 0x62
#define RXB0EID8 0x63
#define RXB0EID0 0x64
#define RXB0DLC 0x65
#define RXB0D0 0x66 

//MCP2515 Command Bytes
#define RESET 0xC0
#define READ 0x03
#define READ_RX_BUFFER 0x90
#define WRITE 0x02
#define LOAD_TX_BUFFER 0x40
#define RTS 0x80
#define READ_STATUS 0xA0
#define RX_STATUS 0xB0
#define BIT_MODIFY 0x05

boolean MCP2515::initCAN(int baudConst)
{
  byte mode;
  
  SPI.begin();
  
  digitalWrite(SLAVESELECT,LOW);
  SPI.transfer(RESET); //Reset cmd
  digitalWrite(SLAVESELECT,HIGH);
  //Read mode and make sure it is config
  delay(100);
  mode = readReg(CANSTAT) >> 5;
  if(mode != 0b100) 
    return false;
	
  return(setCANBaud(baudConst));

}

boolean MCP2515::setCANBaud(int baudConst)
{
  byte brp;
  
  //BRP<5:0> = 00h, so divisor (0+1)*2 for 125ns per quantum at 16MHz for 500K   
  //SJW<1:0> = 00h, Sync jump width = 1
  switch(baudConst)
  {
    case CAN_BAUD_500K: brp = 0; break;
    case CAN_BAUD_250K: brp = 1; break;
    case CAN_BAUD_125K: brp = 3; break;
    case CAN_BAUD_100K: brp = 4; break;
    default: return false;
  }
  digitalWrite(SLAVESELECT,LOW);
  SPI.transfer(WRITE); 
  SPI.transfer(CNF1);
  SPI.transfer(brp & 0b00111111);
  digitalWrite(SLAVESELECT,HIGH);  
  
  //PRSEG<2:0> = 0x01, 2 time quantum for prop
  //PHSEG<2:0> = 0x06, 7 time constants to PS1 sample
  //SAM = 0, just 1 sampling
  //BTLMODE = 1, PS2 determined by CNF3
  digitalWrite(SLAVESELECT,LOW);
  SPI.transfer(WRITE); 
  SPI.transfer(CNF2);
  SPI.transfer(0b10110001);
  digitalWrite(SLAVESELECT,HIGH); 
  
  //PHSEG2<2:0> = 5 for 6 time constants after sample
  digitalWrite(SLAVESELECT,LOW);
  SPI.transfer(WRITE); 
  SPI.transfer(CNF3);
  SPI.transfer(0x05);
  digitalWrite(SLAVESELECT,HIGH); 
  
  //SyncSeg + PropSeg + PS1 + PS2 = 1 + 2 + 7 + 6 = 16
  
  return true;
}


boolean MCP2515::setSleepMode(){//also need to set CANINTE.WAKIE = 1

	//might need to reset CANINTF.WAKIF bit when awaken.

	//REQOP<2:0> = 001
	//ABAT = 0, do not abort pending transmission, or 01 to abort all pending transmit buffers
	//OSM = 0, messages will reattempt transmission, message will only attempt to transmit one time
	//CLKEN = 1, disable output clock
	//CLKPRE = clk/8

	byte mode;
	writeReg(CANINTE, 0b01000000);
	writeReg(CANCTRL, 0b00100000); //puts to sleep mode
 //sets interrupt to awake on activity
	mode = readReg(CANSTAT) >> 5; //bitwise shift to the right 5 bits
	if(mode != 1)
		return false;
	return true;
}


boolean MCP2515::isAwake(){
	byte mode;
	mode = readReg(CANSTAT) >> 5;
	if(mode != 1){ //if canbus is not in sleep mode
		return true;
	}
	else{
		return false; //return false is canbus is still asleep
	}
}

boolean MCP2515::setCANNormalMode()
{
  //REQOP2<2:0> = 000 for normal mode
  //ABAT = 0, do not abort pending transmission
  //OSM = 0, not one shot
  //CLKEN = 1, disable output clock
  //CLKPRE = 0b11, clk/8
  
  byte mode;
  writeReg(CANINTF,0b00000000);
  writeReg(CANCTRL,0b00000111);
  writeReg(CANINTE,0b00000000); //sets interrupt to awake on activity
  //Read mode and make sure it is normal
  mode = readReg(CANSTAT) >> 5;
  if(mode != 0)
    return false; //if CAN is not normal mode, return false
    
  return true; //if CAN is normal mode, return true
  
}




boolean MCP2515::setCANReceiveonlyMode()
{
  //REQOP2<2:0> = 011 for receive-only mode
  //ABAT = 0, do not abort pending transmission
  //OSM = 0, not one shot
  //CLKEN = 1, disable output clock
  //CLKPRE = 0b11, clk/8
  
  byte mode;
  
  writeReg(CANCTRL,0b01100111);//
  //Read mode and make sure it is receive-only
  mode = readReg(CANSTAT) >> 5;
  if(mode != 3)
    return false;
    
  return true;
  
}


boolean MCP2515::getMSG(CANMSG *msg, unsigned long timeout, unsigned long message_addr){
	unsigned long startTime, endTime;
    boolean gotMessage;
    byte val;
    int i;
	int y = 0; //timeout
	while(msg->adrsValue != message_addr){
		startTime = millis();
		endTime = startTime + timeout;
		gotMessage = false;
		while(millis() < endTime)
		{
		  val = readReg(CANINTF);
		  //If we have a message available, read it
		  if(bitRead(val,RX0IF) == 1)
		  {
			gotMessage = true;
			break;
		  }
		}
    
		if(gotMessage)
		{
			  val = readReg(RXB0CTRL);
			  msg->rtr = ((bitRead(val,3) == 1) ? true : false);

			  //Address received from
			  msg->adrsValue = 0;
			  val = readReg(RXB0SIDH);
			  msg->adrsValue |= (val << 3);
			  val = readReg(RXB0SIDL); 
			  msg->adrsValue |= (val >> 5);
			  msg->isExtendedAdrs = ((bitRead(val,EXIDE) == 1) ? true : false);
			  msg->extendedAdrsValue = 0;
			  if(msg->isExtendedAdrs)
			  {
				msg->extendedAdrsValue = (val & 0x03) << 16;
				val = readReg(RXB0EID8);
				msg->extendedAdrsValue |= (val << 8);
				val = readReg(RXB0EID0);
				msg->extendedAdrsValue |= val;
			  }
      
			  //Read data bytes
			  val = readReg(RXB0DLC);
			  msg->dataLength = (val & 0xf); 
			  digitalWrite(SLAVESELECT,LOW);
			  SPI.transfer(READ); 
			  SPI.transfer(RXB0D0);
			  for(i = 0; i < msg->dataLength; i++)
				msg->data[i] = SPI.transfer(0);
			  digitalWrite(SLAVESELECT,HIGH);
      
			  //And clear read interrupt
			  writeRegBit(CANINTF,RX0IF,0);
		}
		else{//only if no message at all is received
			if(y > 7){ //timeout ->this is the reason why everything is turning off
				gotMessage = false;
				break;
			}
			//break; //breaks out of while look
			y++;
		}
	}
	return gotMessage;

}


boolean MCP2515::SNIFF_ALL(CANMSG *msg){//Sniff all messages found in the CAN
	byte val;
	int i;
	val = readReg(CANINTF);
	if(bitRead(val, RX0IF) == 1){
		val = readReg(RXB0CTRL);
		msg->rtr = ((bitRead(val,3) == 1) ? true : false);

		//Address received from
		msg->adrsValue = 0;
		val = readReg(RXB0SIDH);
		msg->adrsValue |= (val << 3);
		val = readReg(RXB0SIDL); 
		msg->adrsValue |= (val >> 5);
		msg->isExtendedAdrs = ((bitRead(val,EXIDE) == 1) ? true : false);
		msg->extendedAdrsValue = 0;
		if(msg->isExtendedAdrs)
		{
		msg->extendedAdrsValue = (val & 0x03) << 16;
		val = readReg(RXB0EID8);
		msg->extendedAdrsValue |= (val << 8);
		val = readReg(RXB0EID0);
		msg->extendedAdrsValue |= val;
		}
      
		//Read data bytes
		val = readReg(RXB0DLC);
		msg->dataLength = (val & 0xf); 
		digitalWrite(SLAVESELECT,LOW);
		SPI.transfer(READ); 
		SPI.transfer(RXB0D0);
		for(i = 0; i < msg->dataLength; i++)
		msg->data[i] = SPI.transfer(0);
		digitalWrite(SLAVESELECT,HIGH);
      
		//And clear read interrupt
		writeRegBit(CANINTF,RX0IF,0);
		return true;
	}
	return false;
}


boolean MCP2515::CANSNIFF(CANMSG *msg, unsigned short address, unsigned long timeout){ //Sniff specific messages for further testing
   unsigned long startTime, endTime;
    boolean gotMessage;
    byte val;
    int i;
	int y = 0; //timeout
	while(msg->adrsValue != address){
		startTime = millis();
		endTime = startTime + timeout;
		gotMessage = false;
		while(millis() < endTime)
		{
		  val = readReg(CANINTF);
		  //If we have a message available, read it
		  if(bitRead(val,RX0IF) == 1)
		  {
			gotMessage = true;
			break;
		  }
		}
    
		if(gotMessage)
		{
			  val = readReg(RXB0CTRL);
			  msg->rtr = ((bitRead(val,3) == 1) ? true : false);

			  //Address received from
			  msg->adrsValue = 0;
			  val = readReg(RXB0SIDH);
			  msg->adrsValue |= (val << 3);
			  val = readReg(RXB0SIDL); 
			  msg->adrsValue |= (val >> 5);
			  msg->isExtendedAdrs = ((bitRead(val,EXIDE) == 1) ? true : false);
			  msg->extendedAdrsValue = 0;
			  if(msg->isExtendedAdrs)
			  {
				msg->extendedAdrsValue = (val & 0x03) << 16;
				val = readReg(RXB0EID8);
				msg->extendedAdrsValue |= (val << 8);
				val = readReg(RXB0EID0);
				msg->extendedAdrsValue |= val;
			  }
      
			  //Read data bytes
			  val = readReg(RXB0DLC);
			  msg->dataLength = (val & 0xf); 
			  digitalWrite(SLAVESELECT,LOW);
			  SPI.transfer(READ); 
			  SPI.transfer(RXB0D0);
			  for(i = 0; i < msg->dataLength; i++)
				msg->data[i] = SPI.transfer(0);
			  digitalWrite(SLAVESELECT,HIGH);
      
			  //And clear read interrupt
			  writeRegBit(CANINTF,RX0IF,0);
		}
	}
	return gotMessage;
}


boolean MCP2515::receiveCANMessage(CANMSG *msg, unsigned long timeout)
{
    unsigned long startTime, endTime;
    boolean gotMessage;
    byte val;
    int i;

    startTime = millis();
    endTime = startTime + timeout;
    gotMessage = false;
    while(millis() < endTime)
    {
      val = readReg(CANINTF);
      //If we have a message available, read it
      if(bitRead(val,RX0IF) == 1)
      {
        gotMessage = true;
        break;
      }
    }
    
    if(gotMessage)
    {
      val = readReg(RXB0CTRL);
      msg->rtr = ((bitRead(val,3) == 1) ? true : false);

      //Address received from
      msg->adrsValue = 0;
      val = readReg(RXB0SIDH);
      msg->adrsValue |= (val << 3);
      val = readReg(RXB0SIDL); 
      msg->adrsValue |= (val >> 5);
      msg->isExtendedAdrs = ((bitRead(val,EXIDE) == 1) ? true : false);
      msg->extendedAdrsValue = 0;
      if(msg->isExtendedAdrs)
      {
        msg->extendedAdrsValue = (val & 0x03) << 16;
        val = readReg(RXB0EID8);
        msg->extendedAdrsValue |= (val << 8);
        val = readReg(RXB0EID0);
        msg->extendedAdrsValue |= val;
      }
      
      //Read data bytes
      val = readReg(RXB0DLC);
      msg->dataLength = (val & 0xf); 
      digitalWrite(SLAVESELECT,LOW);
      SPI.transfer(READ); 
      SPI.transfer(RXB0D0);
      for(i = 0; i < msg->dataLength; i++)
        msg->data[i] = SPI.transfer(0);
      digitalWrite(SLAVESELECT,HIGH);
      
      //And clear read interrupt
      writeRegBit(CANINTF,RX0IF,0);
    }
    
    return gotMessage;
}

boolean MCP2515::transmitCANMessage(CANMSG msg, unsigned long timeout)
{
  unsigned long startTime, endTime;
  boolean sentMessage;
  unsigned short val;
  int i;
 
  startTime = millis();
  endTime = startTime + timeout;
  sentMessage = false;

  val = msg.adrsValue >> 3;
  writeReg(TXB0SIDH,val);
  val = msg.adrsValue << 5;
  if(msg.isExtendedAdrs)
    val |= 1 << EXIDE;
  writeReg(TXB0SIDL,val);
  if(msg.isExtendedAdrs)
  {
    val = msg.extendedAdrsValue >> 8;
    writeReg(TXB0EID8,val);
    val = msg.extendedAdrsValue;
    writeReg(TXB0EID0,val);
  }
  
  val = msg.dataLength & 0x0f;
  if(msg.rtr)
    bitWrite(val,TXRTR,1);
  writeReg(TXB0DLC,val);
  
  //Message bytes
  digitalWrite(SLAVESELECT,LOW);
  SPI.transfer(WRITE); 
  SPI.transfer(TXB0D0);
  for(i = 0; i < msg.dataLength; i++)
    SPI.transfer(msg.data[i]);
  digitalWrite(SLAVESELECT,HIGH);

  //Transmit the message
  writeRegBit(TXB0CTRL,TXREQ,1);

  sentMessage = false;
  while(millis() < endTime)
  {
    val = readReg(CANINTF);
    if(bitRead(val,TX0IF) == 1)
    {
      sentMessage = true;
      break;
    }
  }

  //Abort the send if failed
  writeRegBit(TXB0CTRL,TXREQ,0);
  
  //And clear write interrupt
  writeRegBit(CANINTF,TX0IF,0);

  return sentMessage;

}

byte MCP2515::getCANTxErrCnt()
{
  return(readReg(TEC));
}

byte MCP2515::getCANRxErrCnt()
{
  return(readReg(REC));
}

void MCP2515::writeReg(byte regno, byte val)
{
  digitalWrite(SLAVESELECT,LOW);
  SPI.transfer(WRITE); 
  SPI.transfer(regno);
  SPI.transfer(val);
  digitalWrite(SLAVESELECT,HIGH);  
}

void MCP2515::writeRegBit(byte regno, byte bitno, byte val)
{
  digitalWrite(SLAVESELECT,LOW);
  SPI.transfer(BIT_MODIFY); 
  SPI.transfer(regno);
  SPI.transfer(1 << bitno);
  if(val != 0)
    SPI.transfer(0xff);
  else
    SPI.transfer(0x00);
  digitalWrite(SLAVESELECT,HIGH);
}

byte MCP2515::readReg(byte regno)
{
  byte val;
  
  digitalWrite(SLAVESELECT,LOW);
  SPI.transfer(READ); 
  SPI.transfer(regno);
  val = SPI.transfer(0);
  digitalWrite(SLAVESELECT,HIGH);
  
  return val;  
}  


long MCP2515::queryOBD(unsigned char pid, char *buffer)
{
  CANMSG msg;
  //long val;
  float engine_data;
  boolean rxSuccess;
  int noMatch;

  msg.adrsValue = 0x7Df;
  msg.isExtendedAdrs = false;
  msg.extendedAdrsValue = 0;
  msg.rtr = false;
  msg.dataLength = 8;
  msg.data[0] = 0x02;
  msg.data[1] = 0x01;
  msg.data[2] = pid;
  msg.data[3] = 0;
  msg.data[4] = 0;
  msg.data[5] = 0;
  msg.data[6] = 0;
  msg.data[7] = 0;
  
  if(!transmitCANMessage(msg,300))
    return 0;

  rxSuccess = receiveCANMessage(&msg,300);
  if (rxSuccess) 
  {
    //Check if the PIDs match (in case other messages are also on bus)
	noMatch = 0;
    while(msg.data[2] != pid)
	{
        rxSuccess = receiveCANMessage(&msg,300);
        noMatch++;
        if (!rxSuccess || noMatch >= 5) 
		{
            return 0; // if we dont recv a message / query 5 fails
        }
    }
  }  
  else{ //if message is not recieved return 0;

    return 0;
  }
  //if(msg.data[0] == 3)  // runt time code?
  //  val = msg.data[3];
  //else
  //  val = 256 * msg.data[3] + msg.data[4];

  ///inefficent double double error check lol

  if((msg.adrsValue == PID_REPLY) && (msg.data[2] == pid)) 
  {
	  switch(msg.data[2])
	  {   /* Details from http://en.wikipedia.org/wiki/OBD-II_PIDs */
							
			case ENGINE_COOLANT_TEMP: 	// 	A-40			  [degree C]
				engine_data =  msg.data[3] - 40;
				sprintf(buffer,"%d",(int) engine_data);
				break;

			case ENGINE_RPM:
				engine_data = ((msg.data[3]*256) + msg.data[4])/4;
				sprintf(buffer, "%d", (int)engine_data);
				break;

			case FUEL_LEVEL:
				engine_data = ((100*msg.data[3])/255);
				sprintf(buffer,"%d",(int) engine_data);
				break;
							
			case VEHICLE_SPEED: 		// A				  [km]
				engine_data =  msg.data[3];
				sprintf(buffer,"%d",(int) engine_data);
				break;

			case RUN_TIME:
				engine_data = ((msg.data[3]*256)+msg.data[4]);
				sprintf(buffer,"%d", (int) engine_data);
				break;

			case INTAKE_TEMP:
				engine_data = (msg.data[3]-40);
				sprintf(buffer, "%d",(int) engine_data);
				break;

			case MAF_SENSOR:   			// ((256*A)+B) / 100  [g/s]
				engine_data =  ((msg.data[3]*256) + msg.data[4])/100;
				sprintf(buffer,"%d",(int) engine_data);
				break;

			case O2_VOLTAGE:    		// A * 0.005   (B-128) * 100/128 (if B==0xFF, sensor is not used in trim calc)
				engine_data = msg.data[3]*0.005;
				sprintf(buffer,"%d",(int) engine_data);
				break;

/*			case THROTTLE:				// Throttle Position
				engine_data = ((((msg.data[3]*100)/255)-15)/.7);//Subaru vehicles idle at 15%, max at 85%
				if(engine_data < 0){
					engine_data = 0;
				}
				sprintf(buffer,"%d",(int) engine_data);
				break;
*/					
			case REAL_THROTTLE:
				engine_data = (msg.data[3]*100)/255;
				sprintf(buffer,"%d",(int) engine_data);
				break;
		}

	  return 1;// need 0?
  }
    
  return 0;
}


/*
boolean MCP2515::ACCELERATOR(CANMSG *msg, unsigned long timeout){
	unsigned long startTime, endTime;
    boolean gotMessage;
    byte val;
    int i;
	int y = 0;
	while(msg->adrsValue != 0x410){
		startTime = millis();
		endTime = startTime + timeout;
		gotMessage = false;
		while(millis() < endTime)
		{
		  val = readReg(CANINTF);
		  //If we have a message available, read it
		  if(bitRead(val,RX0IF) == 1)
		  {
			gotMessage = true;
			break;
		  }
		}
    
		if(gotMessage)
		{
			  val = readReg(RXB0CTRL);
			  msg->rtr = ((bitRead(val,3) == 1) ? true : false);

			  //Address received from
			  msg->adrsValue = 0;
			  val = readReg(RXB0SIDH);
			  msg->adrsValue |= (val << 3);
			  val = readReg(RXB0SIDL); 
			  msg->adrsValue |= (val >> 5);
			  msg->isExtendedAdrs = ((bitRead(val,EXIDE) == 1) ? true : false);
			  msg->extendedAdrsValue = 0;
			  if(msg->isExtendedAdrs)
			  {
				msg->extendedAdrsValue = (val & 0x03) << 16;
				val = readReg(RXB0EID8);
				msg->extendedAdrsValue |= (val << 8);
				val = readReg(RXB0EID0);
				msg->extendedAdrsValue |= val;
			  }
      
			  //Read data bytes
			  val = readReg(RXB0DLC);
			  msg->dataLength = (val & 0xf); 
			  digitalWrite(SLAVESELECT,LOW);
			  SPI.transfer(READ); 
			  SPI.transfer(RXB0D0);
			  for(i = 0; i < msg->dataLength; i++)
				msg->data[i] = SPI.transfer(0);
			  digitalWrite(SLAVESELECT,HIGH);
      
			  //And clear read interrupt
			  writeRegBit(CANINTF,RX0IF,0);
		}
		else{
			if(y > 7){ //timeout ->this is the reason why everything is turning off
				gotMessage = false;
				break;
			}
			//break; //breaks out of while look
			y++;
		}
	}
	return gotMessage;
}

boolean MCP2515::BRAKE_PRESSURE(CANMSG *msg, unsigned long timeout){
	unsigned long startTime, endTime;
    boolean gotMessage;
    byte val;
    int i;
	int y = 0;
	while(msg->adrsValue != 0x511){
		startTime = millis();
		endTime = startTime + timeout;
		gotMessage = false;
		while(millis() < endTime)
		{
		  val = readReg(CANINTF);
		  //If we have a message available, read it
		  if(bitRead(val,RX0IF) == 1)
		  {
			gotMessage = true;
			break;
		  }
		}
    
		if(gotMessage)
		{
			  val = readReg(RXB0CTRL);
			  msg->rtr = ((bitRead(val,3) == 1) ? true : false);

			  //Address received from
			  msg->adrsValue = 0;
			  val = readReg(RXB0SIDH);
			  msg->adrsValue |= (val << 3);
			  val = readReg(RXB0SIDL); 
			  msg->adrsValue |= (val >> 5);
			  msg->isExtendedAdrs = ((bitRead(val,EXIDE) == 1) ? true : false);
			  msg->extendedAdrsValue = 0;
			  if(msg->isExtendedAdrs)
			  {
				msg->extendedAdrsValue = (val & 0x03) << 16;
				val = readReg(RXB0EID8);
				msg->extendedAdrsValue |= (val << 8);
				val = readReg(RXB0EID0);
				msg->extendedAdrsValue |= val;
			  }
      
			  //Read data bytes
			  val = readReg(RXB0DLC);
			  msg->dataLength = (val & 0xf); 
			  digitalWrite(SLAVESELECT,LOW);
			  SPI.transfer(READ); 
			  SPI.transfer(RXB0D0);
			  for(i = 0; i < msg->dataLength; i++)
				msg->data[i] = SPI.transfer(0);
			  digitalWrite(SLAVESELECT,HIGH);
      
			  //And clear read interrupt
			  writeRegBit(CANINTF,RX0IF,0);
		}
		else{
			if(y > 7){ //timeout ->this is the reason why everything is turning off
				gotMessage = false;
				break;
			}
			//break; //breaks out of while look
			y++;
		}
	}
	return gotMessage;
}

boolean MCP2515::ABSCAN(CANMSG *msg, unsigned long timeout){
	unsigned long startTime, endTime;
    boolean gotMessage;
    byte val;
    int i;
	int y = 0; //timeout
	while(msg->adrsValue != 0x513){
		startTime = millis();
		endTime = startTime + timeout;
		gotMessage = false;
		while(millis() < endTime)
		{
		  val = readReg(CANINTF);
		  //If we have a message available, read it
		  if(bitRead(val,RX0IF) == 1)
		  {
			gotMessage = true;
			break;
		  }
		}
    
		if(gotMessage)
		{
			  val = readReg(RXB0CTRL);
			  msg->rtr = ((bitRead(val,3) == 1) ? true : false);

			  //Address received from
			  msg->adrsValue = 0;
			  val = readReg(RXB0SIDH);
			  msg->adrsValue |= (val << 3);
			  val = readReg(RXB0SIDL); 
			  msg->adrsValue |= (val >> 5);
			  msg->isExtendedAdrs = ((bitRead(val,EXIDE) == 1) ? true : false);
			  msg->extendedAdrsValue = 0;
			  if(msg->isExtendedAdrs)
			  {
				msg->extendedAdrsValue = (val & 0x03) << 16;
				val = readReg(RXB0EID8);
				msg->extendedAdrsValue |= (val << 8);
				val = readReg(RXB0EID0);
				msg->extendedAdrsValue |= val;
			  }
      
			  //Read data bytes
			  val = readReg(RXB0DLC);
			  msg->dataLength = (val & 0xf); 
			  digitalWrite(SLAVESELECT,LOW);
			  SPI.transfer(READ); 
			  SPI.transfer(RXB0D0);
			  for(i = 0; i < msg->dataLength; i++)
				msg->data[i] = SPI.transfer(0);
			  digitalWrite(SLAVESELECT,HIGH);
      
			  //And clear read interrupt
			  writeRegBit(CANINTF,RX0IF,0);
		}
		else{//only if no message at all is received
			if(y > 7){ //timeout ->this is the reason why everything is turning off
				gotMessage = false;
				break;
			}
			//break; //breaks out of while look
			y++;
		}
	}
	return gotMessage;

}
*/
