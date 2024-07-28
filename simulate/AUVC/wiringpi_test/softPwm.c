/*
 * softPwm.c:
 *	Test of the software PWM driver. Needs 8 LEDs connected
 *	to the Pi - e.g. Ladder board.
 *
 * Copyright (c) 2012-2013 Gordon Henderson.
 ***********************************************************************
 * This file is part of wiringPi:
 *      https://github.com/WiringPi/WiringPi
 *
 *    wiringPi is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    wiringPi is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public License
 *    along with wiringPi.  If not, see <http://www.gnu.org/licenses/>.
 ***********************************************************************
 */

#include <stdio.h>
#include <errno.h>
#include <string.h>

#include <wiringPi.h>
#include <softPwm.h>

#define RANGE		100
#define	NUM_LEDS	  8

// #define	LED	0  // GPIO 17
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


int ledMap [NUM_LEDS] = { 0/*GPIO_17*/, 
			  1/*GPIO_18*/,
			  2/*GPIO_27*/,
			  3/*GPIO_22*/,
			  4/*GPIO_23*/,
			  5/*GPIO_24*/
			  } ;

int values [NUM_LEDS] = { 0, 25, 50, 75, 100, 75, 50, 25 } ;

int main()
{
  int i, j ;
  //char buf [80] ;

  wiringPiSetup ()  ;

  for (i = 0 ; i < NUM_LEDS ; ++i)
  {
    softPwmCreate (ledMap [i], 0, RANGE) ;
    printf ("%3d, %3d, %3d\n", i, ledMap [i], values [i]) ;
  }

  // // fgets (buf, 80, stdin) ;

  // // Bring all up one by one:

  // for (i = 0 ; i < NUM_LEDS ; ++i)
  //   for (j = 0 ; j <= 100 ; ++j)
  //   {
  //     softPwmWrite (ledMap [i], j) ;
  //     delay (10) ;
  //   }

  // // fgets (buf, 80, stdin) ;

  // // All Down

  // for (i = 100 ; i > 0 ; --i)
  // {
  //   for (j = 0 ; j < NUM_LEDS ; ++j)
  //     softPwmWrite (ledMap [j], i) ;
  //   delay (10) ;
  // }

  // fgets (buf, 80, stdin) ;

  for (;;)
  {
    for (i = 0 ; i < NUM_LEDS ; ++i)
      softPwmWrite (ledMap [i], values [i]) ;

    delay (50) ;

    i = values [0] ;
    for (j = 0 ; j < NUM_LEDS - 1 ; ++j)
      values [j] = values [j + 1] ;
    values [NUM_LEDS - 1] = i ;
  }
}
