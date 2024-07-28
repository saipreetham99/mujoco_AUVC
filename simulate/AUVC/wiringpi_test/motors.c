#include <stdio.h>
#include <errno.h>
#include <string.h>

#include <wiringPi.h>
#include <softPwm.h>

// #define	LED	0  // GPIO 17
// #define	LED	1  // GPIO 18
// #define	LED	2  // GPIO 27
// #define	LED	3  // GPIO 22
// #define	LED	4  // GPIO 23
// #define	LED	5  // GPIO 24
// #define	LED     6  // GPIO 25

#define servoPin 0 // signal pin for the ESC.
// byte potentiometerPin = A0; // analog input pin for the potentiometer.
// Servo servo;

int main() {
//servo.attach(servoPin);
if (wiringPiSetup () == -1)
  {
    fprintf (stdout, "oops: %s\n", strerror (errno)) ;
    return 1 ;
  }
 
// pwmSetClock(1920);
// pwmSetRange(200);

pinMode (servoPin, PWM_OUTPUT) ;
// pwmSetMode(PWM_MODE_MS);

delay(7000);

printf("ready\n");
gpioClockSet(servoPin, 50);

//servo.writeMicroseconds(1500); // send "stop" signal to ESC. Also necessary to arm the ESC.
// pwmWrite (1, bright); // TOOD: Test this isntead of the next line

pwmWrite(servoPin, 1500);

delay(7000); // delay to allow the ESC to recognize the stopped signal.
printf("done!\n");

while(1){
    pwmWrite(servoPin,1900); // Send signal to ESC.
    delayMicroseconds(50);
}
return 0;

}
