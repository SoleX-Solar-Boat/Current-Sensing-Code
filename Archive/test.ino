/*
  Arduino Starter Kit example
  Project 11 - Crystal Ball

  This sketch is written to accompany Project 11 in the Arduino Starter Kit

  Parts required:
  - 220 ohm resistor
  - 10 kilohm resistor
  - 10 kilohm potentiometer
  - 16x2 LCD screen
  - tilt switch

  created 13 Sep 2012
  by Scott Fitzgerald

  http://www.arduino.cc/starterKit

  This example code is part of the public domain.
*/

// include the library code:
#include <LiquidCrystal.h>

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

char msg1[16];
char msg2[16];
char msg3[16];


int SP_sensorPin = A0;    // select the input pin for the solar panels current sensor
int Bat_sensorPin = A1;    // select the input pin for the Battery current sensor
int Mot_sensorPin = A2;    // select the input pin for the Motor current sensor


// set up a constant for the tilt switch pin
const int switchPin = 6;

// set up a constant for solar panels current sensor
const int SP_CSensorPin = 7;

// set up a constant for  battery current sensor
const int Bat_CSensorPin = 8;

// set up a constant for motor current sensor
const int Mot_CSensorPin = 9;

// variable to hold the value of the switch pin
int switchState = 0;

// variable to hold previous value of the switch pin
int prevSwitchState = 0;

// variable to count 3 second
unsigned long previousmillis = millis();
unsigned long interval =3000;

// a variable to choose which reply from the crystal ball
int reply;

int SP_currentValue;  // variable to store the value coming from the sensor
int Bat_currentValue;  // variable to store the value coming from the sensor
int Mot_currentValue;  // variable to store the value coming from the sensor

//boolean value to choose the way to change screen 0= pressing button, 1= automatically
boolean Change_screen=0;

void setup() {
  // set up the number of columns and rows on the LCD
  lcd.begin(16, 2);

  // set up the switch pin as an input
  pinMode(switchPin, INPUT);
  pinMode(SP_CSensorPin, INPUT);
  pinMode(Bat_CSensorPin, INPUT);
  pinMode(Mot_CSensorPin, INPUT);

  SP_currentValue = 60;  // variable to store the value coming from the sensor
  Bat_currentValue = 154;  // variable to store the value coming from the sensor
  Mot_currentValue = 98;  // variable to store the value coming from the sensor

  lcd.setCursor(0, 0);
  // print some text
  lcd.print("press the button");
}

void RefreshScreen(int r);

void loop() {
  // check the status of the switch
  switchState = digitalRead(switchPin);

  //read sensors
//  SP_currentValue = analogRead(SP_sensorPin);
//  Bat_currentValue = analogRead(Bat_sensorPin);
//  Mot_currentValue = analogRead(Mot_sensorPin);

  sprintf(msg1, "%d A", SP_currentValue); // buffer to combine a variable with a string
  sprintf(msg2, "%d A", Bat_currentValue); 
  sprintf(msg3, "%d A", Mot_currentValue); 

  if(Change_screen==1){
    if(millis() - previousmillis> interval){
      reply = reply+1;
      lcd.clear();
        if (reply >2){
          reply=0;
    }
    previousmillis = millis();
    RefreshScreen(reply);
    }
  }
  else if (Change_screen==0){
    // compare the switchState to its previous state
    if (switchState != prevSwitchState) {
      // if the state has changed from HIGH to LOW you know that the ball has been
      // tilted from one direction to the other
      if (switchState == LOW) {
        // randomly chose a reply
        reply = reply+1;
        if (reply >2){
          reply=0;
        }
        // clean up the screen before printing a new reply
        lcd.clear();
        RefreshScreen(reply);  
      }
    }
  // save the current switch state as the last state
  prevSwitchState = switchState;
  }  
}

 void RefreshScreen(int r){
        // choose a saying to print based on the value in reply
        switch (r) {
          case 0:
            // set the cursor to column 0, line 0
            lcd.setCursor(0, 0);
            // print some text
            lcd.print("Solar Panels:");
            // move the cursor to the second line
            lcd.setCursor(0, 1);
            lcd.print(msg1);
          break;

          case 1:
            // set the cursor to column 0, line 0
            lcd.setCursor(0, 0);
            // print some text
            lcd.print("Battery current:");
            // move the cursor to the second line
            lcd.setCursor(0, 1);
            lcd.print(msg2);
          break;

          case 2:
            // set the cursor to column 0, line 0
            lcd.setCursor(0, 0);
            // print some text
            lcd.print("Motor Current:");
            // move the cursor to the second line
            lcd.setCursor(0, 1);
            lcd.print(msg3);
          break;
        }
 }
