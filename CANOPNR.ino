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

   -----------------------------------------------------------------------------------------
*/


#include <SoftwareSerial.h>
#include <PString.h>
#include <CANOPNR_MCP2515.h>
#include <MCP2515_defs.h>
#include <SPI.h>
#include <SD.h>
#include <stdio.h>

#define GPSRATE 4800
#define BUFFSIZ 90 // plenty big

Sd2Card card;
SdVolume volume;
SdFile root;
SdFile file;
int CONTROLLER_ID = -1; // Defaults to -1
char conn_str[45] = "AT+CIPSTART=\"TCP\",\"";  //starting empty slot is idx 19
char buffer[128];  //Data will be temporarily stored to this buffer before being written to the file
char stringBuffer[700];
char tempbuff[128];
PString dataString(stringBuffer, sizeof(stringBuffer)); //string to be sent
PString tempbuffS(tempbuff, sizeof(tempbuff));
byte sleepmode = 0x01;
byte normalmode = 0x00;
byte listenmode = 0x03;

SoftwareSerial canbus =  SoftwareSerial(4, 5); // for GPS
SoftwareSerial cell(7, 8);
MCP2515 HSCAN;

char buffidx;

int sleeper = 0;
int power_to = 0;
int close_to = 0;
byte mode;

void setup() {                    // need to change this
  Serial.begin(19200);
  if (card.init(SPI_HALF_SPEED,9) && volume.init(&card) &&
      root.openRoot(&volume) && file.open(root, "config.txt", O_READ)) {
    uint8_t i;	
    i=0;
    while (1) {
      buffer[i] = file.read();
      if (buffer[i] == ';') {
	buffer[i] = '\0';
	break;
      }
      i++;
    } 
    CONTROLLER_ID = atoi(buffer);
    file.read();file.read(); // consume the newline & cr
    i=19;
    while (1) {
      conn_str[i] = file.read();
      if (conn_str[i] == ';') break;
      i++;
    } 
    conn_str[i++] = '"';
    conn_str[i++] = ',';
    conn_str[i++] = '"';
    file.read();file.read(); // consume the newline & cr
    while (1) {
      conn_str[i] = file.read();
      if (conn_str[i] == ';') break;
      i++;
    } 
    conn_str[i++] = '"';
    conn_str[i] = '\0';
    //	  Serial.println(CONTROLLER_ID);
    //	  Serial.println(conn_str);
  }    

  cell.begin(19200);
  canbus.begin(GPSRATE);

  //GPRSPower();

  // Begin the SPI module
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  SPI.begin();     

  disableHSCAN();

  int baudRate = 0;
  if(HSCAN.initCAN(CAN_BAUD_500K)){
    delay(50);
    if(!HSCAN.setCANNormalMode()) { 
      Serial.print("CAN E");
    }
  }
  init_GPRS(); 

}


void loop() {
loop_start:
  int timeo4 = 0, timeo3 = 0, timeo2 = 0, timeo1 = 0;
  enableHSCAN();
  dataString.begin();
  tempbuffS.begin();

  CANMSG message;
  message.adrsValue = 0x001;
start_check:
  cell.println(conn_str); //Open a connection to server
  if(cell_wait_for_bytes(12,100) == 0)//what happens if timeouts occur, do we try again?
  {  
    if(power_to > 1){//timed out 2 times, assuming GPRS is off
      power_to = 0; 
      init_GPRS(); //initialize GPRS
      goto loop_start; //reloop
    }
    Serial.println("T1");
    cell.println("AT+CIPSHUT"); //Close the GPRS Connection    
    delay(100);
    power_to++;
    goto loop_start;
  }
  else
  {
    power_to = 0; //clear power timeout, message was received
    while(cell.available()!=0)
    {
      tempbuffS.print((unsigned char)cell.read());
    }
    if(strstr(tempbuffS,"ERROR") != NULL || strstr(tempbuffS,"FAIL") != NULL){//if ERROR exists in tempbuffS
      Serial.println("E1");
      timeo1 += 1; //timeout counter for error 1
      tempbuffS.begin(); //clear tempbuffS
      cell.println("AT+CIPSHUT"); //Close the GPRS Connection
      if(cell_wait_for_bytes(4,100) == 0){ 
        init_GPRS();//didnt need to reinit_GPRS even when errored, just relooped
        Serial.println("ET1");
      }
      if(timeo1 > 4){
        init_GPRS(); 
        timeo1 = 0;
      }
      Serial.print("1:");
      Serial.println(timeo1);
      goto start_check;
    }
    else{
      tempbuffS.begin();
    }
  } 
  dataString.print(CONTROLLER_ID);
  while(strncmp(tempbuffS, "$GPRMC",6) != 0){ //while GPRS does not find GPRMC and a location, it will loop in here
    //strncmp != 0 if not found
    //strchr = NULL if not found
    readline(); //grabs GPS DATA stores to gpsbuffer
  }
  dataString.print(tempbuffS);
  dataString.print("|"); 
  //  dataString.print("0");
sleep_check:
  if(HSCAN.getMSG(&message,100,ACCELERATOR)){ //the the code is stuck in the while loop inside ACCELERATOR
    dataString.print(message.data[4],DEC); //prints the message
    //    Serial.print("Current CANSTAT: ");
    //    Serial.println(HSCAN.readReg(CANSTAT),HEX);
  } 
  else{
    sleeper++;
    //    Serial.print("Sleep: ");
    //    Serial.println(sleeper);
    if(sleeper >= 5){ //5 attempts have gone, non successful
      while(HSCAN.setSleepMode() == false){
        ; //put CANBUS to sleep, but arduino will still be alive
        delay(1000);//if not successful, delay 1 second, then retry
        //         Serial.println(HSCAN.readReg(CANSTAT), HEX);
      }
      //       Serial.println(HSCAN.readReg(CANSTAT), HEX);
      //if asleep, need to stay in here
      //Serial.println("SP");
      cell.println("AT+CIPSHUT"); //shutdown any connections
      delay(400);
      GPRSPower(); //turn off GPRS
      while(HSCAN.isAwake() == false){ //while sleeping
        delay(2500);  //check if awake every 2.5 seconds
        //         Serial.println("StillSleeping");
      }
      //       Serial.println("Awake");
      while(HSCAN.setCANNormalMode() == false){
        ;
        delay(1000);
        //once it wakes up, it will be in listenmode only, this ensures it goes back to normal mode
      }
      //       Serial.print(HSCAN.readReg(CANSTAT), DEC);
      //       Serial.println("<--Awake: 0");
      GPRSPower(); //power on GPRS
      init_GPRS(); //initialize GPRS after turning it on from sleep
      sleeper = 0; //reset sleeper time out
      goto loop_start; //go back to loop start
    }
    goto sleep_check; //sleeper is not > 2, but no accelerator pedal message was received
  }
  sleeper = 0; //data was received, reset sleeper timeout
  dataString.print("|");    //seperate data
  for(int i = 0; i <= 18; i++){
    message.adrsValue = 0x001; //resets address to get a different wheel speed
    if(HSCAN.getMSG(&message, 100, ABSCAN) && i%2 == 0)//checks if its an even number
    {
      dumpMessage(&message);
      dataString.print("*"); //star seperated wheel speed from brake_pressure
      if(HSCAN.getMSG(&message, 100, BRAKE_PRESSURE)){//if wheel speed is found, need to find corresponding brake_pressure with it
          dataString.print(message.data[4],DEC);//print out brake pressure in 
      }
      //else if(!HSCAN.BRAKE_PRESSURE){
      //goto loop_start;// means wheel speed was found but not brake pressure 
      //  }
    }
    //else if(!HSCAN.ABSCAN(&message,100)){
    //  goto loop_start; //clear messages, go back to the beginning
    //}
    if(i<18 && i%2 == 0){
      dataString.print("*");
    }
  }
  //  dataString.print("00#00#00#00#00#00#00#00+00#00#00#00#00#00#00#00+00#00#00#00#00#00#00#00+00#00#00#00#00#00#00#00+00#00#00#00#00#0#00#00+00#00#00#00#00#00#00#00+00#00#00#00#00#00#00#00+00#00#00#00#00#00#00#00+00#00#00#00#00#00#00#00+00#00#00#00#00#00#00#00");
  dataString.print("|");
  if(HSCAN.queryOBD(ENGINE_RPM,buffer) == 1){
    dataString.print(buffer);    
  }
  dataString.print("|"); //RPM

  if(HSCAN.queryOBD(VEHICLE_SPEED,buffer) == 1){
    dataString.print(buffer); //prints an INT to the buffer     
  }  
  dataString.print("|");
  if(HSCAN.queryOBD(ENGINE_COOLANT_TEMP,buffer) == 1){
    dataString.print(buffer);    
  }
  dataString.print("|");//Throttle
  if(HSCAN.queryOBD(FUEL_LEVEL, buffer) == 1){
    dataString.print(buffer);
  }
  dataString.print("|");    
  if(HSCAN.queryOBD(RUN_TIME, buffer) == 1){
    dataString.print(buffer); 
  }
  dataString.print("|"); 
  if(HSCAN.queryOBD(INTAKE_TEMP, buffer) == 1){
    dataString.print(buffer); 
  }
  dataString.print("|"); 
  if(HSCAN.queryOBD(MAF_SENSOR, buffer) == 1){
    dataString.print(buffer);
  }
  dataString.print("|"); 
  if(HSCAN.queryOBD(O2_VOLTAGE, buffer) == 1){
    dataString.print(buffer); 
  }

  disableHSCAN();
  dataString.print("|"); //Strength
  cell.println("AT+CSQ");
  if(cell_wait_for_bytes(12,75) == 0){
    dataString.print("NA");
  }
  else{
    tempbuffS.begin();
    while(cell.available()!=0){
      tempbuffS.print((unsigned char)cell.read());
    }
  }
  cell.flush();
  //need to figure out a way to remove +CSQ ___ from the string and add to dataString
  char* sigptr = strchr(tempbuffS, '+CSQ '); //returns a pointer to 'c' in tempbuffS 
  sigptr += 1; //returns the signal strength, next 4 characters in this array
  char sigstrength[5];
  sigstrength[0] = *sigptr;
  sigstrength[1] = *(sigptr+1);
  sigstrength[2] = *(sigptr+2);
  sigstrength[3] = *(sigptr+3);
  sigstrength[4] = NULL;
  dataString.print(sigstrength);//signal strength

    tempbuffS.begin();

  delay(400);
send_check:
  cell.println("AT+CIPSEND"); //Start data through TCP connection
  if(cell_wait_for_bytes(5,100) == 0)
  {  
    Serial.println("T2");//timeout on CIPSend
    cell.println("AT+CIPSHUT");
    delay(100);
    goto loop_start;
  }
  else
  {
    while(cell.available()!=0)
    {
      tempbuffS.print((unsigned char)cell.read());
    }
    if(strstr(tempbuffS,"ERROR") != NULL || strstr(tempbuffS,"FAIL") != NULL){//if ERROR exists in tempbuffS
      tempbuffS.begin(); //clear tempbuffS
      Serial.println("E2");
      timeo2 += 1;
      if(timeo2 > 4){
        cell.println("AT+CIPSHUT"); //Close the GPRS Connection
        delay(100);
        init_GPRS(); //reinitialize GPRS
        goto loop_start; //reloop
      }
      Serial.print("2:");
      Serial.println(timeo2);
      goto send_check;
    }
    else{
      tempbuffS.begin();
    }
  }

  //Serial.println(dataString);

  delay(15);
  cell.print(dataString);
  delay(100);
  cell.print("\r\n"); 
  delay(100);
  cell.print("\r\n"); 
  delay(100);
end_check:

  cell.write(0x1A); // ARDUINO 1.0 doesn't support "BYTE" -- cell.print(0x1A,BYTE);//not getting this character   
  
  if(cell_wait_for_bytes(20,255) == 0)
  {  
    Serial.println("T3");
    cell.println("AT+CIPSHUT");
    if(cell_wait_for_bytes(4,100) == 0){
      init_GPRS();
    } //Close the GPRS Connection    
    goto loop_start;
  }
  else
  {
    while(cell.available()!=0)
    {
      tempbuffS.print((unsigned char)cell.read());
    }
    if(strstr(tempbuffS,"ERROR") != NULL || strstr(tempbuffS,"FAIL") != NULL){//if ERROR exists in tempbuffS
      tempbuffS.begin(); //clear tempbuffS
      Serial.println("E3");
      timeo3 += 1;
      if(timeo3 > 4){
        cell.println("AT+CIPSHUT"); //Close the GPRS Connection
        delay(100);
        init_GPRS(); 
        goto loop_start;
      }
      Serial.print("3:");
      Serial.println(timeo3);
      goto end_check;
    }
  }
close_check:
  cell.println("AT+CIPCLOSE=0"); //Close the GPRS Connection
  if(cell_wait_for_bytes(7,100) == 0)
  {  
    close_to++;
    if(close_to >= 3){ //attempted to close the connection 3 times, unsuccessful    
      init_GPRS(); //automatically goes to loop start right after
    }
    else{
      goto close_check;
    }
  }       
}

void readline(void) {
  char c;

  buffidx = 0; // start at begninning
  while (1) {
    c=canbus.read();
    if (c == -1)
      continue;
    if (c == '\n')
      continue;
    if ((buffidx == BUFFSIZ-1) || (c == '\r')) {
      tempbuff[buffidx] = 0;
      return;
    }
    tempbuff[buffidx++]= c;
  }
}

char cell_wait_for_bytes(char no_of_bytes, int timeout) //timeout amount is *100 MS
{
  while(cell.available() < no_of_bytes)
  {
    delay(100);
    timeout-=1;
    if(timeout == 0)
    {
      return 0;
    }
  }
  return 1;
}
void init_GPRS(){
setup_start:
  Serial.println();
  tempbuffS.begin();

  cell.println("AT");
  if(cell_wait_for_bytes(2,100) == 0)
  {   //wait one second for a response
    Serial.println("P was off");
    GPRSPower();
    goto setup_start;
  }
  else{
    Serial.println("P OK");
  }

  cell.flush();

  cell.println("AT+CIPSHUT");
  delay(150);

  cell.println("AT+CIPMUX=0"); //We only want a single IP Connection at a time.
  if(cell_wait_for_bytes(4,100) == 0)
  {  
    Serial.println("T4");
    goto setup_start;
  }
  else
  {
    while(cell.available()!=0)
    {
      tempbuffS.print((unsigned char)cell.read());
    }
    if(strstr(tempbuffS,"ERROR") != NULL){ //means it found ERROR in the string
      Serial.println("E4");
      cell.println("AT+CIPSHUT");
      delay(100);
      goto setup_start;
    }
    else{
      tempbuffS.begin();
    }
  }        

  cell.println("AT+CIPMODE=0"); //was AT+CIPMODE=1 for one connection. Selecting "Normal Mode" and NOT "Transparent Mode" as the TCP/IP Application Mode
  if(cell_wait_for_bytes(4,100) == 0)
  {  
    Serial.println("T5");
    goto setup_start;
  }
  else
  {
    while(cell.available()!=0)
    {
      tempbuffS.print((unsigned char)cell.read());
    }
    if(strstr(tempbuffS,"ERROR") != NULL){ //means it found ERROR in the string
      Serial.println("E5");
      cell.println("AT+CIPSHUT");
      delay(100);      
      goto setup_start;
    }
    else{
      tempbuffS.begin();
    }
  }

  cell.println("AT+CGDCONT=1,\"IP\",\"web.gci\""); //Defining the Packet Data
  if(cell_wait_for_bytes(4,100) == 0)
  {  
    Serial.println("T6");
    goto setup_start;
  }
  else
  {
    while(cell.available()!=0)
    {
      tempbuffS.print((unsigned char)cell.read());
    }
    if(strstr(tempbuffS,"ERROR") != NULL){ //means it found ERROR in the string
      Serial.println("E6");
      cell.println("AT+CIPSHUT");
      delay(100);      
      goto setup_start;
    }
    else{
      tempbuffS.begin();
    }

  }


  cell.println("AT+CSTT=\"WEB.GCI\""); //Start Task and set Access Point Name (and username and password if any)
  if(cell_wait_for_bytes(4,100) == 0)
  {  
    Serial.println("T7");
    goto setup_start;
  }
  else
  {
    while(cell.available()!=0)
    {
      tempbuffS.print((unsigned char)cell.read());
    }
    if(strstr(tempbuffS,"ERROR") != NULL){ //means it found ERROR in the string
      Serial.println("E7");
      cell.println("AT+CIPSHUT");
      delay(100);    
      goto setup_start;
    }
    else{
      tempbuffS.begin();
    }
  }
  cell.println("AT+CIPSHUT");
  delay(100);

}

void dumpMessage(CANMSG* message) {
  for(int i = 0; i <= message->dataLength - 1 ; i++) {
    dataString.print(message->data[i], HEX);
    if(i < message->dataLength - 1){
      dataString.print("#");
    }
  }
}

/**
 * Initialize the SPI pins for both CAN busses
 */
void init_SPI_CS(void)
{
  pinMode(9,OUTPUT);
  digitalWrite(9, HIGH);
}  
/**
 * Initialize the status LED
 */
/**
 * Enable the HSCAN by setting is pin low
 */
void enableHSCAN() {
  digitalWrite(10, LOW); 
}
/**
 * Disable CAN device by setting its
 * pins high
 */
void disableHSCAN() {
  digitalWrite(10, HIGH); 
}
void GPRSPower(){
  pinMode(9, OUTPUT); 
  digitalWrite(9,LOW);
  delay(1000);
  digitalWrite(9,HIGH);
  delay(2500);
  digitalWrite(9,LOW);
  delay(3500);
}



// pin 10: slave select
// pin 11: master out slave in
// pin 12: master in slave out
// pin 13: serial clock

