void setup() {
  Serial.begin(9600);  // Initialize serial communication at 9600 baud
}

void loop() {
  if (Serial.available() > 0) {
    char receivedChar = Serial.read();  // Read the incoming byte
    Serial.print(receivedChar);         // Echo the received byte
  }
  
  delay(10);  // Small delay to prevent overwhelming the serial buffer
}

