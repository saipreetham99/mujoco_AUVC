#include <Servo.h>

byte servoPin1 = 9;
byte servoPin2 = 10;
byte servoPin3 = 12;
byte servoPin4 = 4;
byte servoPin5 = 5;
byte servoPin6 = 6;

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;
Servo servo6;

void x(int input_val){	
    int val = input_val + 1500;
    int neg_val = 1500 - input_val;
    servo1.writeMicroseconds(neg_val); 
    servo2.writeMicroseconds(1500); 
    servo3.writeMicroseconds(val); 
    servo4.writeMicroseconds(val); 
    servo5.writeMicroseconds(neg_val); 
    servo6.writeMicroseconds(1500);
}
void y(int input_val){	
    int val = input_val + 1500;
    int neg_val = 1500 - input_val;
    servo1.writeMicroseconds(val); 
    servo2.writeMicroseconds(1500); 
    servo3.writeMicroseconds(neg_val); 
    servo4.writeMicroseconds(val); 
    servo5.writeMicroseconds(neg_val); 
    servo6.writeMicroseconds(1500);
}

void z(int input_val){	
    int val = input_val + 1500;
    int neg_val = 1500 - input_val;
    servo1.writeMicroseconds(1500); 
    servo2.writeMicroseconds(neg_val); 
    servo3.writeMicroseconds(1500); 
    servo4.writeMicroseconds(1500); 
    servo5.writeMicroseconds(1500); 
    servo6.writeMicroseconds(val);
}

void setup() {
  
 Serial.begin(9600);
 servo1.attach(servoPin1);
 servo2.attach(servoPin2);
 servo3.attach(servoPin3);
 servo4.attach(servoPin4);
 servo5.attach(servoPin5);
 servo6.attach(servoPin6);

 servo1.writeMicroseconds(1500); 
 servo2.writeMicroseconds(1500); 
 servo3.writeMicroseconds(1500); 
 servo4.writeMicroseconds(1500); 
 servo5.writeMicroseconds(1500); 
 servo6.writeMicroseconds(1500); 

 delay(7000); // delay to allow the ESC to recognize the stopped signal
}

void loop() {
  
  Serial.println("Enter PWM signal value 1100 to 1900, 1500 to stop");
  
  while (Serial.available() == 0);
  
  int val = Serial.parseInt(); 
  
  if (val == 0){
	while(Serial.available()==0);
	val = Serial.parseInt();
	x(val);
	}
  if (val == 1){
	while(Serial.available()==0);
	val = Serial.parseInt();
	y(val);
	} 
  if (val == 2){
	while(Serial.available()==0);
	val = Serial.parseInt();
	z(val);
	}
  delay(5000); 
}
