//Simple Current sensor testing code - K. Lister-Grotz & M. Richmond
//25.02.2022

float sensorPin = A0; //sensor pin A0 reads analog data from current sensor (decimal val)

void setup() {
  Serial.begin(9600); 
}

void loop() {
  int Value = analogRead (sensorPin);  //variable "Value" corresponds to sensor pin readings (intager val)
  
  Serial.println(Value);  //display sensor reading as serial data
  delay(500);             //delay 1/2 second
    
}
