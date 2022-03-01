//Simple Current sensor testing code - K. Lister-Grotz & M. Richmond
//01.03.2022

float sensorPin = A0; //sensor pin A0 reads analog data from current sensor (decimal val)
float vpp = 1024/5;
int Ipn = 200; //Primary nominal current of sensor (200 A)
int Uc = 5; // Input voltage from Arduino to sensor

void setup() {
  Serial.begin(9600); 
}

void loop() {
  int Value = analogRead (sensorPin);  //variable "Value" corresponds to sensor pin readings (intager val)
  float Vout = (Value/vpp); // convert digital reading to analogue voltage (0-5 V)
  float Ip = (((Vout-(0.5*Uc))/1.25)*Ipn);

  
  Serial.println(Ip);  //display sensor reading as serial data
  delay(500);             //delay 1/2 second
    
}
>>>>>>> de5005340c4957f18e9808d2d7c3513778290e45:CurrentTest2.ino
