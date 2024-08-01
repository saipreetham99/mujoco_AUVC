#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <MPU9250.h>

#include "main.h"

// Set I2C bus to use: Wire, Wire1, etc.
#define WIRE Wire // SDA  18; SCL  19;
Servo m1;
Servo m2;
Servo m3;
Servo m4;
Servo m5;
Servo m6;
Servo led;

MPU9250 mpu;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  delay(2000);

  Serial.println("");
  Serial.println(" .... Main Control Code .... ");
  setupMotorsAndLed();

  // Send IMU Data;
  setupIMU();
  Serial.print()

  if(Serial.available() == 0){
    int val = Serial.parseInt();
  }else{
    Serial.println("Serial Not available!");
  }
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

int setupMotorsAndLed(){
  Serial.println("Setting up Motors");
	m1.attach(mPin1);
	m2.attach(mPin2);
	m3.attach(mPin3);
	m4.attach(mPin4);
	m5.attach(mPin5);
	m6.attach(mPin6);

	led.attach(mPin6);

	m1.writeMicroseconds(1500); // send "stop" signal to ESC.
	m2.writeMicroseconds(1500); // send "stop" signal to ESC.
	m3.writeMicroseconds(1500); // send "stop" signal to ESC.
	m4.writeMicroseconds(1500); // send "stop" signal to ESC.
	m5.writeMicroseconds(1500); // send "stop" signal to ESC.
	m6.writeMicroseconds(1500); // send "stop" signal to ESC.
	led.writeMicroseconds(1100); // LED Setup

	delay(7000); // delay to allow the ESC to recognize the stopped signal
}

void fireLed(){
  led.writeMicroseconds(1700);
}

void setupIMU(){
  if (!mpu.setup(0x68)) {  // change to your own address
      Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
      delay(5000);
  }
}

void ornControl(){

}
