#include <LiquidCrystal.h>
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.print("Ready");
}

void loop() {
  float sensorPin = A0;    // select the input pin for the current sensor
  float A0_Value = analogRead(sensorPin);
  int Max_Value = 1024;
  int Sat_Voltage = 5;
  float Reference = Max_Value / Sat_Voltage
  float Vout = A0_Value/Reference; //Converting measure analogue to digatal voltage value
  float Vref = 2.5; //Read as live value
  float Ip = (((Vout - Vref)/1.25) * 200); 
  //float Ip = Vref + ((1.25*Vout)/200); 
  //float Ip = ((200*(Vout-Vref))/1.25); //Converting Vout to Ip

  
  Serial.println(Ip);

  // set the cursor to column 0, line 0
  lcd.setCursor(0, 0);
  // print some text
  lcd.print("Ip (A)");
  // move the cursor to the second line
  lcd.setCursor(0, 1);
  lcd.print(Ip);


  
  delay(1000);
}




void measuring() {


}





void calibration() {


}