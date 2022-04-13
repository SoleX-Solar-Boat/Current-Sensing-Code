#include <SoftwareSerial.h>
//#include <iostream>

//-----------------------------------------------------------//
/**
	Serial communication Setup
*/
int pinRX = 11;
int pinTX = 10;
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
int delayAfterSendingMessage = 100;
float countOFAllBufferResets = 0.00;
float countOfAllMessagesCorreclyBounded = 0.00;
float bufferResetMax = 400000000.00; //4294967040

//Sensor output calibration
int Analog_V_Range = 5;
int Analog_Bit_Range = 1024;



void setup()  
{
  Serial.begin(softwareBaudRate);
  Serial.flush();
  softSerial.begin(softwareBaudRate);
  softSerial.flush();
} 



void loop()  
{ 
  /Digital reading
	int Digital_Sensor_Reading = analogRead (A0);

  //Digtial to Analog converter
  double conversion_factor = Analog_Bit_Range/Analog_V_Range;
  double Analog_Sensor_Reading = Digital_Sensor_Reading/conversion_factor;

  //Voltage to Current Reading
  double Current = (((Analog_Sensor_Reading-Vref)/1.25)*Sensor_Ipn);
  Serial.print("Reading");
  Serial.println(Reading);
  
  int Int_Value = Reading * 100.00 ;//shifting first 2dp left, and converting to int 
  String message = String(Int_Value);
  
  
  char ID_Marker = SolarOutput_ID;
  softSerial.print(charStartMarker);
  softSerial.print(ID_Marker);
  softSerial.print(message);
  softSerial.print(charEndMarker);
  
  Serial.print("sent ");
  Serial.println(message);
  
  delay(delayAfterSendingMessage);
  
}