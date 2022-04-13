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

void recv_Wraped_Message(){

	static boolean recvInProgress = false;
	static byte ndx = 0;

	boolean newData = false;
	newInstruction = false;
	boolean messageStarted = false;

	char charReadFromSerial;
	

	while (softSerial.available() > 0 && newData == false) {
		charReadFromSerial = (char) softSerial.read();

		if (recvInProgress == true) {
			if (charReadFromSerial != charEndMarker) {
				receivedChar[ndx] = charReadFromSerial;
				ndx++;
				if (ndx >= numBytes) {
					ndx = numBytes - 1;
				}
			} 
			else {
				//countMessageNumber(messageStarted);
				receivedChar[ndx] = '\0'; // terminate the string
				recvInProgress = false;
				numReceived = ndx; // save the number for use when printing
				ndx = 0;
				newData = true;
				newInstruction = true;
             	              
			}
        } 
		 else if (charReadFromSerial == charStartMarker) {
           		memset(receivedChar, 0, sizeof receivedChar);
				recvInProgress = true;
				messageStarted = true;
		}
	}
}



void loop() 
{
  recv_Wraped_Message();
  //Verfication
  int Test = sizeof(receivedChar);
  Serial.println(receivedChar);
  //if (sizeof(receivedChar) == 5){
  //    Serial.write(receivedChar);
  //}
  
}
 