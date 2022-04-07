//Electrical System Arduino Code  - K. Lister-Grotz & M. Richmond

#include <LiquidCrystal.h>
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);


void setup() {
  //initialise serial communication at 9600 bits per second:
  Serial.begin(9600);

  //initialisng and setting up LCD
  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.print("Ready");
}

//-----------------------------------------------------------------------------------------------------

//Control Loop
void loop() {
  Output();
  delay(500); //delay 1/2 second between taking readings

}

//-----------------------------------------------------------------------------------------------------

//Calibrative Values (Future work to change this to active code)
class Calibration {
public:
  char State;

  int Analoug_Max_V;
  int Analoug_Max_Bit;

  float HallEffect1_Pin = A0; 
  float HallEffect2_Pin;
  float HallEffect3_Pin;
};

//-----------------------------------------------------------------------------------------------------

//Sensing Class - Takes Readings from relevant ports and processess
class Sensing {
public:
  int Value;

void Read() 
   {
       Value = analogRead (Calibration::HallEffect1_Pin);
   }
};

//-----------------------------------------------------------------------------------------------------

//Outputing and displaying vValues
void Output() {

  int Value = Sensing::Value;

  //Returing value along serial port
  Serial.println(Value);

  //Displaying Value on LCD
  lcd.setCursor(0, 0);
  lcd.print("Value");
  lcd.setCursor(0, 1);
  lcd.print(Value);
}

//-----------------------------------------------------------------------------------------------------