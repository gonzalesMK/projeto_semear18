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

    Serial.begin(115200);

    digitalWrite(IN1A, 1);
    digitalWrite(IN1B, 0);
    analogWrite(ENABLE1, 0);

    digitalWrite(IN2A, 1);
    digitalWrite(IN2B, 0);
    analogWrite(ENABLE2, 0);

    digitalWrite(IN3A, 1);
    digitalWrite(IN3B, 0);
    analogWrite(ENABLE3, 0);

    digitalWrite(IN4A, 1);
    digitalWrite(IN4B, 0);
    analogWrite(ENABLE4, 0);
    
}

void loop()
{

}

void serialEvent()
{
    if( Serial.available() < 4){
        return;
    }

    Serial.readBytes(PWM, 4);

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
