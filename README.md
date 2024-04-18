# RFM73Lib
Author: Benedikt Simma
Date: 20.3.2024

Library for the RFM73 Chip
This Library is designed to work with the RFM73 Chip by HOPERF 
and to be used in combination with an Arduino AT2560 and its integrated SPI-Interface.

Datasheet: https://cdn.soselectronic.com/productdata/d2/4d/4a7068e5/rfm73-s.pdf

The Goal of this Library is the implementation of all the functionalities of the chip.
This Library was made in the scope of a school project and may be missing some features 
and may not live up to any standards or norms.

Missing functions will most likley not be implemented and this Library is considered finished.

Examples:

Here are simple examples for use as receiver and sender. They are meant to be used togehter.

Receiver:

This is an Example for a simple receiver, which prints all received data into the Serial Monitor of the ArduinoIDE
 ```
#include <rfm73.h>

void setup(){

  //Serial connection

  Serial.begin(19200);

  //Initializes Chip USE your connected Pins here
  
  RFM.initChip();  

  RFM.onReceive(receiveEvent);
}

void loop(){ 

  //delays 1ms to guarantee the correct use of checknewData

  delay(1);

  //checks for new data by reading the DataReceived Flag

  RFM.checknewData();
  
}

void receiveEvent(void){

  //prints the length and content of receiver data

  Serial.print("Length: ");

  Serial.print(RFM.getPacketLength());

  Serial.print("\tContent: ");

  Serial.println((char*)RFM.getRcvBuffer());

}
 ```


Sender:

This is an Example for a simple sender, which send the data saved in array b every second
An additional function is used to fill the array b with the variable a.
 ```
#include <rfm73.h>

#include <stdio.h>

#include <stdlib.h>

#include <string.h>

void setup(){

  //Serial connection

  Serial.begin(19200);

  //Initializes Chip USE your connected Pins here

  RFM.initChip();
}


byte array[32] = "12345678901234567890123456789012";

void loop(){ 

  int a = 122; 

  byte b[6] = "a";

  writeValueToArray(a,b,6);

  //sends the array b with a length of 6

  RFM.send(b, 6);

  //delay to allow readability on the receiver side

  delay(1000);
}

void writeValueToArray(int value, char* array, int arrayLength) {

    char valueStr[20];

    snprintf(valueStr, sizeof(valueStr), "%d", value);

    int valueLength = strlen(valueStr);

    strcpy(array, valueStr);

}
 ```