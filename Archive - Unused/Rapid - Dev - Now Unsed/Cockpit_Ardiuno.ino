//Recvies sent messages as indiviudla bytes "<S22>"
//ToDO
//	-Recive into array that can then be translated into real value with identifier, '<' and '>' used to verifiy message end and start




#include <SoftwareSerial.h>
//-----------------------------------------------------------//
/**
	Serial communication Setup
*/
int pinRX = 10;
int pinTX = 11;
SoftwareSerial softSerial(pinRX, pinTX);


//message marker to help with verfication
const char charStartMarker = '<'; //<
const char charEndMarker = '>'; //>
const char SolarOutput_ID = 'S'; //S
const char BatteryInput_ID = 'B'; //B
const char MotorInput_ID = 'M'; //M

const byte numBytes = 8;
char receivedChar[numBytes];

//int numBytes = 6;
//char receivedChar[numBytes];
byte numReceived = 0;
boolean newData = false;
boolean newInstruction = false;

/**
	 variables for serial communication
*/
int softwareBaudRate = 4800;



void setup() {
  Serial.begin(softwareBaudRate);
  softSerial.begin(softwareBaudRate);
  
}

void loop() 
{
  while (softSerial.available() > 0) {
    char inByte = softSerial.read();
    Serial.write(inByte);
    softSerial.flush();
  }
}
 