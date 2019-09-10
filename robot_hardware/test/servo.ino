#include "Arduino.h"
#include "pins_arduino.h"
#include <Servo.h>

#define SERVO_PIN 6

Servo servo;

void setup()
{
    Serial.begin(9600);

    servo.attach(SERVO_PIN);
}

int servo_pose;

void loop()
{
}

void serialEvent()
{

    int ch = Serial.read();

    switch (ch)
    {
    case 1: // Liga controle do servo e pede por posição
        servo_pose = -1;

        while (servo_pose == -1)
        {
            if (Serial.available())
            {
                servo_pose = (int)Serial.read();
            }
        }

        servo.write(servo_pose);
        break;
    }
}
