 /**
	Current Sensing, Processing and Display Code - K. Lister-Grotz & M. Richmond
	03.05.2022
	Notes:
		-Before debugging::: CHECK PIN VALUES, CHECK COMMON GROUND!!!!!
		-Issue when sensor digital value = 0, so when current massivley negative that exceeds sensor range (i think line 268, if Recived_Value = 0 MSG_Size =0 when should =1)
	ToDo
		-LCD_Output
		-Self Calibration, need access to solar boat completed electronics

	If your reading this and trying to understand my logic in places just understand, it works! Function before form, if you want to make it 
	neat and tidy go ahead but hopefully its pretty self explaintory. There are some overarching issues namley the ammount of times i switch datatypes
	for the same peice of information, its frankly hilarious and should be adressed, but hey it works. Like i think the messages are read as Char, become
	strings then ints then maybe dobule, at this point im too afraiad to actually work out how badly i've screwed up, but it works. 

*/

#include <Arduino.h>
#include <SoftwareSerial.h>
#include<math.h>

//-----------------------------------------------------------//
/**
	*Values and variables for use through out the project
*/
float Solar_Curret;
float Battery_Curret;
float Motor_Curret;

//-----------------------------------------------------------//
/**
	Hall Effect Sensor Calibration
	HallEffect_"SensorName"_Pin = Pin connected to on board
	HallEffect_"SensorName"_NominalCurrent = 
*/
int HallEffect_SolarOutput_Pin = A0; 
int HallEffect_BatteryInput_Pin = A1;
int HallEffect_MotorInput_Pin = A2;

//Sensor Primary Nominal rms current
int HallEffect_SolarOutput_NominalCurrent = 100; 
int HallEffect_BatteryInput_NominalCurrent = 200;
int HallEffect_MotorInput_NominalCurrent = 200;

//Sensor output calibration
int Analog_V_Range = 5;
int Analog_Bit_Range = 1024;

//-----------------------------------------------------------//
/**
	Serial communication Setup
	RX on Sensor connects to TX on Cockpit
	TX on sensor connects to RX in Cockpit
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
byte numReceived = 0;
boolean newData = false;
boolean newInstruction = false;

//variables for serial communication
int softwareBaudRate = 9600;
int delayAfterSendingMessage = 1000;
float countOFAllBufferResets = 0.00;
float countOfAllMessagesCorreclyBounded = 0.00;
float bufferResetMax = 400000000.00; //4294967040

//-----------------------------------------------------------//
/**
	LCD setup - No longer usable purley example, taken from arduino example LCD circuit
	  The circuit:
 	* LCD RS pin to digital pin 12
 	* LCD Enable pin to digital pin 11
 	* LCD D4 pin to digital pin 5
 	* LCD D5 pin to digital pin 4
 	* LCD D6 pin to digital pin 3
 	* LCD D7 pin to digital pin 2
 	* LCD R/W pin to ground
 	* LCD VSS pin to ground
 	* LCD VCC pin to 5V
 	* 10K resistor:
	* ends to +5V and ground
 	* wiper to LCD VO pin (pin 3)
*/
//#include <LiquidCrystal.h>
//const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
//LiquidCrystal lcd(rs, en, d4, d5, d6, d7);


//-----------------------------------------------------------//
/**
	Equation for converting the analog volatge value into a current reading for the LEM Current Transducer HTFS200...800p
	Vout = Vref +-(1.25*Ip/Ipn)
	Vout = Sensor Voltage
	Ip = Current
	Ipn = Sensor Primary Nominal rms current
*/
double LEM_HTFS_Current(int Digital_Sensor_Reading, int Vref, int Ipn){
	//Digtial to Analog converter
	double conversion_factor = Analog_Bit_Range/Analog_V_Range;
	double Analog_Sensor_Reading = Digital_Sensor_Reading/conversion_factor;
	//Voltage to Current Reading
	double Current = (((Analog_Sensor_Reading-Vref)/1.25)*Ipn);
	return Current;
}

//-----------------------------------------------------------//
/**
	Equation of LEM_HASS sensor
	Vout = Vref +-(0.625*Ip/Ipn)
	Vout = Sensor Voltage
	Ip = Current
	Ipn = Sensor Primary Nominal rms current
*/
double LEM_HASS_Current(int Digital_Sensor_Reading, int Vref, int Ipn){

	//Digtial to Analog converter
	double conversion_factor = Analog_Bit_Range/Analog_V_Range;
	double Analog_Sensor_Reading = Digital_Sensor_Reading/conversion_factor;
	//Voltage to Current Reading
	double Current = (((Analog_Sensor_Reading-Vref)/0.625)*Ipn);
	return Current;
}

//-----------------------------------------------------------//
/**
	Can check if int within a range
*/
bool TrueINT(int val, int minimum, int maximum)
{
  return ((minimum <= val) && (val <= maximum));
}

//-----------------------------------------------------------//
/**

*/
void setup() {
	Serial.begin(softwareBaudRate);
	Serial.flush();
	softSerial.begin(softwareBaudRate);
	softSerial.flush();

	// set up the LCD's number of columns and rows:
  	//lcd.begin(16, 2); for when LCD is re-installed
}

//-----------------------------------------------------------//
/**
	JUST HERE for when calibration class is written
*/
void calibration(){
}

//-----------------------------------------------------------//
/**
	OLD, works for display individual reading on small LCD, just here as example
	Currently not called anywhere
*/
void LCD_Output(){
	lcd.setCursor(0, 0);
	lcd.print("Solar Panels:");
	lcd.setCursor(0, 1);
	lcd.print(Solar_Curret);
}

//-----------------------------------------------------------//
/**
	Sends message with Id maker, Length marker, start and end markers
*/
void Wrap_Send_Message(int Reading, char ID_Marker){
	String message = String(Reading); //Convert int reading into string

	//determing length of msg so can be used for verification on recving ardiuno
	int length;
	if (Reading >= 1000){
		length = 4;
	}
	if (Reading >= 100 && Reading <= 999){
		length = 3;
	}
	if (Reading >=10 && Reading <=99){
		length = 2;
	}
	if (Reading >= 0 && Reading <=9){
		length = 1;
	}
	//Sending message
	softSerial.print(charStartMarker);
	softSerial.print(length);
	softSerial.print(ID_Marker);
	softSerial.print(message);
	softSerial.print(charEndMarker);
	delay(delayAfterSendingMessage); //Trust me, probaly could decrease the time but when no delay chance of message corruption and/or message skip increases
}

//-----------------------------------------------------------//
/**
	Receives a message wrapped using start and end markers to identify contents source: J.Hinselwood
*/
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

//-----------------------------------------------------------//
/**
	Controls the sensor arduino located in electronics bay and connected to the 
	hall effect sensors, simple read value (analogRead) and send message (Wrap_Send_Message)
*/
void Sensor_Arduino() {
	//Read data and send Messages
	Wrap_Send_Message(analogRead (HallEffect_SolarOutput_Pin), SolarOutput_ID);
	Wrap_Send_Message(analogRead (HallEffect_MotorInput_Pin), MotorInput_ID);
	Wrap_Send_Message(analogRead (HallEffect_BatteryInput_Pin), BatteryInput_ID);
}

//-----------------------------------------------------------//
/**
	Control cockpit arduino, includes message extraction, verification and assignment to relevant sensor
*/
void Cockpit_Arduino(){
	double Vref = 2.5;

	recv_Wraped_Message();

	char Recived_Length = receivedChar[0]; 
	char Recived_ID = receivedChar[1];
	char Recived_MSG[numBytes];
	memset(Recived_MSG, 0, sizeof Recived_MSG);
	int Recived_Value = 0; //int version of Recived_MSG
	char digit; //just a placeholder thing for use on ln290, could probably get rid off?
	
	bool corruput = false;
	//will exit shoud at anytime the message be corrupt
	while (corruput == false){

		//First Corrupt check, see of Sensor ID is one of the allowed
		if (Recived_ID == SolarOutput_ID || Recived_ID == BatteryInput_ID || Recived_ID == MotorInput_ID) {
			corruput = false;
		}
		else{
			corruput = true;
		}

		//Extract msg
		for (int i = 0; i <= sizeof(receivedChar); i++){
		digit = receivedChar[i + 2]; //+ 2 as first 2 Chracters of recivedchar corrospond to length and sensor ID
			if (TrueINT(digit, 48, 57)){ //Ascii 48 = 0, Ascii 57 = 9, the cause of this is a lot of different data types used throughout
				Recived_MSG[i] = digit;
				corruput = false;
			}
			else {
				//This is here becuase if using corruput = false; then cannont enter assigning procedure, should a corrupt value be found it exits out and will be handled by lenght check, yes it could be better but i have yet to figure out a way
				break;
			}
		}

		//Verify msg length
		Recived_Value = atoi(Recived_MSG); //Char[] to int
		int MSG_Size = trunc(log10(Recived_Value)) + 1; //extracting size of recived message 
		String Recived_Length_String = String(Recived_Length);// converting a constant char into a String
		if (MSG_Size == Recived_Length_String.toInt()){
			corruput = false;
		}
		else{
			corruput = true;
		}
		break;
	}

	//Assinging of extracted value to correct global variable
	if (corruput == false){
		if (Recived_ID == 'S'){
			Solar_Curret = LEM_HTFS_Current(Recived_Value, Vref, HallEffect_SolarOutput_NominalCurrent);
		}
		else if (Recived_ID == 'B'){
			Battery_Curret = LEM_HTFS_Current(Recived_Value, Vref, HallEffect_BatteryInput_NominalCurrent);
		}
		else if (Recived_ID == 'M'){
			Motor_Curret = LEM_HTFS_Current(Recived_Value, Vref, HallEffect_MotorInput_NominalCurrent);
		}
	}
	//send to lcd
}

//-----------------------------------------------------------//
/**
	Runs the Arduino, chose between Cockpit_Arduino or Sensor_Arduino
*/
void loop() {
	Cockpit_Arduino();
		//or
	//Sensor_Arduino();
}