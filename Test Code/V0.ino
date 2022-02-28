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
  int sensorPin = A0;    // select the input pin for the current sensor
  int A0_Value = analogRead(sensorPin);
  Serial.println(A0_Value);

  // set the cursor to column 0, line 0
  lcd.setCursor(0, 0);
  // print some text
  lcd.print("Sensor Value");
  // move the cursor to the second line
  lcd.setCursor(0, 1);
  lcd.print(A0_Value);


  
  delay(1000);
}