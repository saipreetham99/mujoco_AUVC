#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <MPU9250.h>

// Set I2C bus to use: Wire, Wire1, etc.
#define WIRE Wire // SDA  18; SCL  19;
Servo m1;
Servo m2;
Servo m3;
Servo m4;
Servo m5;
Servo m6;
Servo LedPin;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  while (!Serial)
     delay(10);
  Serial.println("\nMain Control Code");
}

void loop() {
	Serial.println("Enter PWM signal value 1100 to 1900, 1500 to stop");
  if(Serial.available()){
    char receivedChar = Serial.read();  // Read the incoming byte
    Serial.print(receivedChar);         // Echo the received byte
  }
  delay(10);  // Small delay to prevent overwhelming the serial buffer

	// int val = Serial.parseInt();
	// //int val = 1100;
	// if(val < 1100 || val > 1900)
	// {
	// 	Serial.println("not valid");
	// }
	// else
	// {
	// 	forward(val);
	// }
}


int setupMotors(){
  Serial.println("hi");
}
