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
#define IS_BLACK 200
#define FL 0
#define FR 1
#define BL 3
#define BR 2
#define LF 5
#define LB 4
#define RF 6
#define RB 7
#define FL_black 55
#define FR_black 190
#define RF_BLACK 75
#define RB_BLACK 300
#define BL_BLACK 47
#define BR_BLACK 42
#define LF_BLACK 42 // bigger is green
#define LR_BLACK 45 // bigger is green

uint8_t ordering[] = {FL, FR, BL,BR, LF, LB, RF, RB};
// SENSORES POLOLU
QTRSensors qtr;
bool publish_sensors = false;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];


void setup()
{
    Serial.begin(9600);

    pinMode(CONTROL_PIN, OUTPUT);
    digitalWrite(CONTROL_PIN, HIGH);  
    qtr.setTypeAnalog();
    qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);
    qtr.setEmitterPin(CONTROL_PIN);
    qtr.emittersOff();
    qtr.setDimmingLevel(0);
    qtr.emittersOn();
}

void loop()
{
    sensorValues[0] = analogRead(A0);
    sensorValues[1] = analogRead(A1);
    sensorValues[1] = analogRead(A1);
    sensorValues[3] = analogRead(A3);
    sensorValues[3] = analogRead(A3);
    sensorValues[2] = analogRead(A2);
    sensorValues[2] = analogRead(A2);
    sensorValues[4] = analogRead(A4);
    sensorValues[4] = analogRead(A4);
    sensorValues[5] = analogRead(A5);
    sensorValues[5] = analogRead(A5);
    sensorValues[6] = analogRead(A6);
    sensorValues[6] = analogRead(A6);
    sensorValues[7] = analogRead(A7);
    sensorValues[7] = analogRead(A7);

    byte sensorsValuesByte = 0;
/**/
    Serial.print("FL:");
    Serial.print(sensorValues[ordering[0]]);
    Serial.print(" FR:");
    Serial.print(sensorValues[ordering[1]]);
/**
    Serial.print(" BL:");
    Serial.print(sensorValues[ordering[2]]);
    Serial.print(" BR:");
  
    Serial.print(sensorValues[ordering[3]]);
/**
    Serial.print(" LF:");
    Serial.print(sensorValues[ordering[4]]);
    Serial.print(" LB:");
    Serial.print(sensorValues[ordering[5]]);
/**
    Serial.print(" RF:");
    Serial.print(sensorValues[ordering[6]]);
    Serial.print(" RB:");
    Serial.print(sensorValues[ordering[7]]);
/**
    Serial.print("    ");    
    
    for (uint8_t i = 0; i < SensorCount; i++)
    {
        sensorsValuesByte |= ((sensorValues[ordering[i]] > IS_BLACK) << i);
        Serial.print( (int) (  sensorValues[ordering[i]] > IS_BLACK)  );
    }
    
    Serial.print( " Char value: ");
    Serial.print(sensorsValuesByte);
   
/**/
    Serial.println();
    delay(5);
}
