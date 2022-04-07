  while (softSerial.available() > 0) {
    char inByte = softSerial.read();
    Serial.write(inByte);
    softSerial.flush();