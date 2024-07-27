#!/bin/sh
set -x

gcc blink.c -L/usr/local/lib -lwiringPi -I/usr/local/include -Wall -pipe -Winline -o blink.out
gcc softServo.c servo.c -L/usr/local/lib -lwiringPi -I/usr/local/include -Wall -pipe -Winline -o servo.out
gcc softPwm.c -L/usr/local/lib -lwiringPi -I/usr/local/include -Wall -pipe -Winline -o softPwm.out

