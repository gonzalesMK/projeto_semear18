/*

*/
#include "Arduino.h"
#include "pins_arduino.h"

#define IN1A
#define IN1B
#define ENABLE1

#define IN2A
#define IN2B
#define ENABLE2

#define IN3A
#define IN3B
#define ENABLE3

#define IN4A
#define IN4B
#define ENABLE4

char PWM1=0;
char PWM2=0;
char PWM3=0;
char PWM4=0;

void setup() {
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

    Serial.begin(9600);

}

void loop() {
    
}

void serialEvent() {
    PWM1 = (char) Serial.read();

    while(!Serial.available()){}
    
    PWM2 = (char) Serial.read();

    while(!Serial.available()){}
    
    PWM3 = (char) Serial.read();
    
    while(!Serial.available()){}

    PWM4 = (char) Serial.read();

    digitalWrite( IN1A , PWM1 > 0);
    digitalWrite( IN1B , PWM1 < 0);
    analogWrite( ENABLE1, (unsigned char) abs(PWM1) * 2);

    digitalWrite( IN2A , PWM2 > 0);
    digitalWrite( IN2B , PWM2 < 0);
    analogWrite( ENABLE2, (unsigned char) abs(PWM2) * 2);

    digitalWrite( IN3A , PWM1 > 0);
    digitalWrite( IN3B , PWM1 < 0);
    analogWrite( ENABLE3, (unsigned char) abs(PWM3) * 2);

    digitalWrite( IN4A , PWM1 > 0);
    digitalWrite( IN4B , PWM1 < 0);
    analogWrite( ENABLE4, (unsigned char) abs(PWM4) * 2);
}