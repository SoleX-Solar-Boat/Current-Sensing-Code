/**
	Current Sensing, Processing and Display Code - K. Lister-Grotz & M. Richmond
	28.03.2022

	Notes:
		-

	ToDo:
		-Self Calibration, need access to solar boat completed electronics
		-LCD_Output
		-Cockpit_Arduino, data extraction, verfication, storage and processing
*/
#include <Arduino.h>
#include <SoftwareSerial.h>


//-----------------------------------------------------------//
/**
	*Fixed Values for use through out the project
*/
bool Calibration_State;


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

/**
	Reading sensor digital value and then converting to current value
	Equation for converting the analog volatge value into a current reading for the LEM Current Transducer HTFS200...800p
	Vout = Vref +-(1.25*Ip/Ipn)
	Vout = Sensor Voltage
	Ip = Current
	Ipn = Sensor Primary Nominal rms current
*/
double LEM_HTFS_Current(int Sensor_Ipn, int Sensor_Pin, int Sensor_Vref){

	//Digital reading
	int Digital_Sensor_Reading = analogRead (sensorPin);

	//Digtial to Analog converter
	double conversion_factor = Analog_Bit_Range/Analog_V_Range;
	double Analog_Sensor_Reading = Digital_Sensor_Reading/conversion_factor;

	//Voltage to Current Reading
	double Current = (((Analog_Sensor_Reading-Vref)/1.25)*Sensor_Ipn);


	return Current
}


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
void Wrap_Send_Message(String message, char ID_Marker){


	softSerial.print(charStartMarker);
	softSerial.print(ID_Marker);
	softSerial.print(message);
	softSerial.print(charEndMarker);

	Serial.print("sent ");
	Serial.println(message);

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
	Converting 2dp Double to integer string
*/
int Double_To_5DString(double Value_To_Convert){
	int Int_Value = Value_To_Convert * 100 ;//shifting first 2dp left, and converting to int 
	String String_Value = String(Int_Value)

	return String_Value
} 

//-----------------------------------------------------------//
/**
	Placeholder
*/
void Sensor_Arduino() {

	double Vref = 2.5

	//LEM_HTFS_Current(int Sensor_Ipn, int Sensor_Pin, int Sensor_Vref)
	double SolarOutput_Current = LEM_HTFS_Current(HallEffect_SolarOutput_NominalCurrent, HallEffect_SolarOutput_Pin, Vref);
	double MotorInput_Curent = LEM_HTFS_Current(HallEffect_MotorInput_NominalCurrent, HallEffect_SolarOutput_Pin, Vref);
	double BatteryInput_Current =LEM_HTFS_Current(HallEffect_BatteryInput_NominalCurrent, HallEffect_BatteryInput_Pin, Vref);

	//Send Messages
	Wrap_Send_Message(Double_To_5DString(SolarOutput_Current), SolarOutput_ID)
	delay(delayAfterSendingMessage)

	Wrap_Send_Message(Double_To_5DString(MotorInput_Curent), MotorInput_ID)
	delay(delayAfterSendingMessage)

	Wrap_Send_Message(Double_To_5DString(BatteryInput_Current), BatteryInput_ID)
	delay(delayAfterSendingMessage)

}


//-----------------------------------------------------------//
/**
	Placeholder
*/
void Cockpit_Arduino(){

	recv_Wraped_Message()

	//Verfication and data extraction


}


//-----------------------------------------------------------//
/**
	Choose which one your running
*/
void loop() {

	//Cockpit_Arduino()
		//or
	Sensor_Arduino()
	
}