/* Esse código é responsável para testar se o arduino é capaz de ler os sensores da Pololu. Mais especificamente, a placa dos sensores é responsável 
 * pela leitura de 8 sensores infravermelhos.
 *  
 *  
 *  Objetivo: ler e printar a leitura dos sensores
 *  
 *  
 */

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

    byte sensorsValuesByte = 0;

    Serial.print("A0:");
    Serial.print(sensorValues[0]);
    Serial.print(" A1:");
    Serial.print(sensorValues[1]);
    Serial.print(" A2:");
    Serial.print(sensorValues[2]);
    Serial.print(" A3:");
    Serial.print(sensorValues[3]);
    Serial.print(" A4:");
    Serial.print(sensorValues[4]);
    Serial.print(" A5:");
    Serial.print(sensorValues[5]);
    Serial.print(" A6:");
    Serial.print(sensorValues[6]);
    Serial.print(" A7:");
    Serial.print(sensorValues[7]);
    
    Serial.print("    ");    
    
    for (uint8_t i = 0; i < SensorCount; i++)
    {
        sensorsValuesByte |= ((sensorValues[i] > IS_BLACK) << i);
        Serial.print( (int) (sensorValues[i] > IS_BLACK)  );
    }
    Serial.print( " Char value: ");
    Serial.print(sensorsValuesByte);
    Serial.println();

    delay(500);
}
