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

char PWM[4];

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

    Serial.begin(115200);
    
}

void loop()
{
}

void serialEvent()
{
    if( Seria.available() < 4){
        return;
    }

    Serial.readBytes(PWM, 4)

    digitalWrite(IN1A, PWM[0] > 0);
    digitalWrite(IN1B, PWM[0] < 0);
    analogWrite(ENABLE1, (unsigned char)abs(PWM[0]) * 2);

    digitalWrite(IN2A, PWM[1] > 0);
    digitalWrite(IN2B, PWM[1] < 0);
    analogWrite(ENABLE2, (unsigned char)abs(PWM[1]) * 2);

    digitalWrite(IN3A, PWM[2] > 0);
    digitalWrite(IN3B, PWM[2] < 0);
    analogWrite(ENABLE3, (unsigned char)abs(PWM[2]) * 2);

    digitalWrite(IN4A, PWM[3] > 0);
    digitalWrite(IN4B, PWM[3] < 0);
    analogWrite(ENABLE4, (unsigned char)abs(PWM[3]) * 2);
}