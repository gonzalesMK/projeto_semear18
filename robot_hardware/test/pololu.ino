#include <QTRSensors.h>

#include "Arduino.h"
#include "pins_arduino.h"

// SENSORES Analógicos POLOLU de A0 até A7
#define CONTROL_PIN 5
#define IS_BLACK 500

// SENSORES POLOLU
QTRSensors qtr;
bool publish_sensors = false;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];


void setup()
{
    Serial.begin(9600);

    qtr.setTypeAnalog();
    qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);
    qtr.setEmitterPin(CONTROL_PIN);
}

void loop()
{
    qtr.readLineBlack(sensorValues);

    if (publish_sensors)
    {
        byte sensorsValuesByte = 0;
        for (uint8_t i = 0; i < SensorCount; i++)
        {
            sensorsValuesByte |= ((sensorValues[i] > IS_BLACK) << i);
        }

        Serial.print(sensorsValuesByte);
        Serial.println();

        delay(500);
    }
}

void serialEvent()
{

    int ch = Serial.read();

    switch (ch)
    {
    case 1: // Liga sensores de linha
        publish_sensors = true;
        break;

    case 2: // Desliga sensores de linha
        publish_sensors = false;
        break;
    }
}
