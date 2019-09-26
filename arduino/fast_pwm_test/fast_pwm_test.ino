#include "Arduino.h"
#include "pins_arduino.h"


// BL
#define IN4A A1
#define IN4B A0
#define ENABLE4 10 

// FL
#define IN2A A2
#define IN2B A3
#define ENABLE2 9

// BR
#define IN3A 4
#define IN3B 3
#define ENABLE3 6

// FR
#define IN1A 7 
#define IN1B 8
#define ENABLE1 5

char PWM[4] = {100,100,100,100};

void setup()
{
    pinMode(IN1A, OUTPUT);
    pinMode(IN1B, OUTPUT);
    pinMode(ENABLE1, OUTPUT);

    pinMode(IN2A, OUTPUT);
    pinMode(IN2B, OUTPUT);
    pinMode(ENABLE2, OUTPUT);

    pinMode(IN3A, OUTPUT);
    pinMode(IN3B, OUTPUT);
    pinMode(ENABLE3, OUTPUT);

    pinMode(IN4A, OUTPUT);
    pinMode(IN4B, OUTPUT);
    pinMode(ENABLE4, OUTPUT);

    digitalWrite(IN1A, 1);
    digitalWrite(IN1B, 0);
    digitalWrite(IN2A, 1);
    digitalWrite(IN2B, 0);
    digitalWrite(IN3A, 1);
    digitalWrite(IN3B, 0);
    digitalWrite(IN4A, 1);
    digitalWrite(IN4B, 0);
    
    
    TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM10);
    // TCCR1A &= (~_BV(WGM11)) & (~_BV(COM1A0)) & (~_BV(COM1B0)) ; 
    
    TCCR1B =  _BV(WGM12) | _BV(CS12) | _BV(CS10); // _BV(ICNC1) noise filter

    
    // COM0A1 = 1 e COM0A0 = 0 -> NON-INVERTING FAST-PWM on OC0A pin
    // COM0B1 = 1 e COM0B0 = 0 -> NON-INVERTING FAST-PWM on OC0B pin
    
    // WGM02 = 1 e WGM01 = 1 e WGM00 = 1 -> Fast PWM, control TOP = OCRA e TOV Flag is SET on TOP
    //              or 
    // WGM02 = 0 e WGM01 = 1 e WGM00 = 1 -> Fast PWM, control TOP = 0xFF e TOV Flag is SET on MAX  <-
    TCCR0A = _BV(COM0B1) | _BV(COM0A1) | _BV( WGM01) | _BV(WGM00);
    TCCR0B = _BV(CS02) | _BV(CS00);

    OCR1A = 255;
    OCR1B = 255;
    OCR0A = 255;
    OCR0B = 255;

    
}

void loop()
{
  analogWrite(ENABLE1, 100);
  analogWrite(ENABLE2, 100);
  analogWrite(ENABLE3, 100);
  analogWrite(ENABLE4, 100);
}
