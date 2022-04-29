 /**
	Current Sensing, Processing and Display Code - K. Lister-Grotz & M. Richmond
	28.03.2022

	Notes:
		-

	ToDo:
		-Self Calibration, need access to solar boat completed electronics
		-LCD_Output
		-Extract message broken
		-Convert raw into current value
*/
#include <Arduino.h>
#include <SoftwareSerial.h>
#include<math.h>


//-----------------------------------------------------------//
/**
	*Values and variables for use through out the project
*/
bool Calibration_State = false;
float Solar_Curret;
float Battery_Curret;
float Motor_Curret;


//-----------------------------------------------------------//
/**
	*Hall Effect Sensor Calibration
	*HallEffect_Location_Pin, where location = PLACEHOLDER
*/
int HallEffect_SolarOutput_Pin = A0; 
int HallEffect_BatteryInput_Pin = A1;
int HallEffect_MotorInput_Pin = A2;

//Sensor Primary Nominal rms current
int HallEffect_SolarOutput_NominalCurrent = 200; 
int HallEffect_BatteryInput_NominalCurrent = 200;
int HallEffect_MotorInput_NominalCurrent = 200;

//Sensor output calibration
int Analog_V_Range = 5;
int Analog_Bit_Range = 1024;


//-----------------------------------------------------------//
/**
	Serial communication Setup
*/
int pinRX = 0;
int pinTX = 1;
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
int softwareBaudRate = 9600;
int delayAfterSendingMessage = 1000;
float countOFAllBufferResets = 0.00;
float countOfAllMessagesCorreclyBounded = 0.00;
float bufferResetMax = 400000000.00; //4294967040


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
	Placeholder
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
	Placeholder
*/
void setup() {
	Serial.begin(softwareBaudRate);
	Serial.flush();
	softSerial.begin(softwareBaudRate);
	softSerial.flush();
}

//-----------------------------------------------------------//
/**
	Placeholder
*/
void calibration(){

}

//-----------------------------------------------------------//
/**
	Placeholder
*/
void LCD_Output(){

}

//-----------------------------------------------------------//
/**
	PlaceHolder
*/
void Wrap_Send_Message(int Reading, char ID_Marker){

	int length;
	String message = String(Reading);


	if (Reading >= 100){
		length = 3;
	}
	if (Reading >=10 && Reading <=99){
		length = 2;
	}
	if (Reading >= 0 && Reading <=9){
		length = 1;
	}


	softSerial.print(charStartMarker);
	softSerial.print(length);
	softSerial.print(ID_Marker);
	softSerial.print(message);
	softSerial.print(charEndMarker);

	delay(delayAfterSendingMessage);
}

//-----------------------------------------------------------//
/**
	Placeholder
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
	Placeholder
*/
void Sensor_Arduino() {

	//Read data and send Messages
	Wrap_Send_Message(analogRead (HallEffect_SolarOutput_Pin), SolarOutput_ID);
	delay(delayAfterSendingMessage);

	Wrap_Send_Message(analogRead (HallEffect_MotorInput_Pin), MotorInput_ID);
	delay(delayAfterSendingMessage);

	Wrap_Send_Message(analogRead (HallEffect_BatteryInput_Pin), BatteryInput_ID);
	delay(delayAfterSendingMessage);

}

//-----------------------------------------------------------//
/**
	Placeholder
*/
void Cockpit_Arduino(){

	double Vref = 2.5;

	recv_Wraped_Message();

	char Recived_Length = receivedChar[0];
	char Recived_ID = receivedChar[1];
	char Recived_MSG[numBytes - 1];
	memset(Recived_MSG, 0, sizeof Recived_MSG);
	int Recived_Value = 0;


	//through into while loop
	bool corruput = false;
	while (corruput = false){
		//First Corrupt check
		if (Recived_ID == 'S' || Recived_ID == 'B' || Recived_ID == 'M') {
			corruput = false;
		}
		else{
			corruput = true;
		}

		//Extract msg (not working!!)
		for (int i = 0; i <= 3; i++){
		char digit = receivedChar[i + 1];
			if (TrueINT(digit, 48, 57)){
				Recived_MSG[i] = digit;
				corruput = false;
			}
			else {
				corruput = true;
			}
		}

		//Verify msg length
		Recived_Value = atoi(Recived_MSG);
		int MSG_Size = trunc(log10(Recived_Value)) + 1;
		if (MSG_Size == Recived_Length){
			corruput = false;
		}
		else{
			corruput = true;
		}
		break;
	}

	if (corruput = false){
		if (Recived_ID == 'S'){
			Solar_Curret = Recived_Value;
		}
		else if (Recived_ID == 'B'){
			Battery_Curret = Recived_Value;
		}
		else if (Recived_ID == 'M'){
			Motor_Curret = Recived_Value;
		}
	}
<<<<<<< HEAD

=======
>>>>>>> parent of f58c09b (add debug stuff in cockpit ardunion code)
}


//-----------------------------------------------------------//
/**
	Choose which one your running
*/
void loop() {

	//Cockpit_Arduino();
		//or
	Sensor_Arduino();
	
}