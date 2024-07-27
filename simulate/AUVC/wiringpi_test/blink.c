#include <stdio.h>
#include <wiringPi.h>

// Refer: https://github.com/WiringPi/WiringPi/blob/master/examples/pwm.c
// 	  The library is also from there. Simply clone, cd <dir>  and ./build
      
// LED Pin - wiringPi pin 0 is BCM_GPIO 17.

// NOTE: If you stop the execution of the program when the
// 	 LED is in the on or the off state, it will remain
// 	 in that state.

// Compile:
// gcc blink.c -L/usr/local/lib -lwiringPi -I/usr/local/include -Wall -pipe -Winline


#define	LED	0  // GPIO 17
// #define	LED	1  // GPIO 18
// #define	LED	2  // GPIO 27
// #define	LED	3  // GPIO 22
// #define	LED	4  // GPIO 23
// #define	LED	5  // GPIO 24
// #define	LED     6  // GPIO 25
// #define	LED     7  // GPIO  4
// #define	LED     8  // GPIO  2 NOTE: This is SDA_1
// #define	LED     9  // GPIO  3 NOTE: This is SCL_1
// #define	LED     10 // GPIO  8 NOTE: SPI
// #define	LED     11 // GPIO  7 NOTE: SPI
// #define	LED     12 // GPIO 10 NOTE: SPI
// #define	LED     13 // GPIO  9 NOTE: SPI
// #define	LED     14 // GPIO 11 NOTE: SPI
// #define	LED     15 // GPIO 14 NOTE: This is UART TxD
// #define	LED     16 // GPIO 15 NOTE: This is UART RxD
// #define	LED     17 // GPIO  NOTE DEFINED IN PIN HEADER IO
// #define	LED     18 // GPIO  NOTE DEFINED IN PIN HEADER IO
// #define	LED     19 // GPIO  NOTE DEFINED IN PIN HEADER IO
// #define	LED     20 // GPIO  NOTE DEFINED IN PIN HEADER IO
// #define	LED     21 // GPIO  5
// #define	LED     22 // GPIO  6
// #define	LED     23 // GPIO 13
// #define	LED     24 // GPIO 19
// #define	LED     25 // GPIO 26
// #define	LED     26 // GPIO 12
// #define	LED     27 // GPIO 16
// #define	LED     28 // GPIO 20
// #define	LED     29 // GPIO  21
// #define	LED     30 // GPIO  NOTE: DNC (Left)  [SDA?]
// #define	LED     31 // GPIO  NOTE: DNC (Right) [SCL?]
// Not Tested Further!

int main (void)
{
  printf ("Raspberry Pi blink\n") ;

  wiringPiSetup () ;
  pinMode (LED, OUTPUT) ;
  pinMode (LED, INPUT) ;

  for (;;)
  {
    digitalWrite (LED, HIGH) ;	// On
    delay (500) ;		// mS
    digitalWrite (LED, LOW) ;	// Off
    delay (500) ;
  }
  return 0 ;
}
