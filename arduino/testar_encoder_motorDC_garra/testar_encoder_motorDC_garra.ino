/* Esse código é responsável para testar se o arduino é capaz de ler encoder. Mais especificamente, a placa dos sensores é responsável por controlar 
 * o motor DC da garra.
 *  
 *  
 *  Objetivo: testar os encoders nos pinos 2 e 3 printando os resultados conforme o motor gira.
 *  
 *  Para fazer o motor girar,, basta abrir o monitor serial e digitar:
 *  0: para
 *  1: sentido horário
 *  2: sentido anti-horário
 *  
 */

#include "Arduino.h"
#include "pins_arduino.h"

// Encoder 
#define encoder1A 2 // D2
#define encoder1B 3 // D3
#define bitEncoder1A (1 << PCINT18)
#define bitEncoder1B (1 << PCINT19)

// Motor
#define IN1 11
#define IN2 9
#define ENABLE 12

const double countsPerRevolution = 8245.92;

// Encoder
volatile double encoder_tick = 0;

void encoder1A_cb()
{
  if (PIND & bitEncoder1B)
    encoder_tick++;
  else
    encoder_tick--;
}

void encoder1B_cb()
{
  if (PIND & bitEncoder1A)
    encoder_tick--;
  else
    encoder_tick++;
}


void setup()
{
  Serial.begin(9600);

  // Encoder
  pinMode(encoder1A, INPUT);
  pinMode(encoder1B, INPUT);

  // Motor
  pinMode( IN1, OUTPUT);
  pinMode( IN2, OUTPUT);
  pinMode( ENABLE, OUTPUT);
  
  delay(1000);

  attachInterrupt(digitalPinToInterrupt(encoder1A), encoder1A_cb, RISING); // D2
  attachInterrupt(digitalPinToInterrupt(encoder1B), encoder1B_cb, RISING); // D3

}

void loop()
{
  Serial.print("Encoder Ticks: ");
  Serial.print(encoder_tick);
  Serial.print("    # of Turns: ");
  Serial.print(encoder_tick/countsPerRevolution);
  Serial.println();
  delay(500);
}


void serialEvent()
{
  char ch = (char)Serial.read();

  switch(ch){

    case '0':
        analogWrite(ENABLE, 0);
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
        break;

    case '1':
        analogWrite(ENABLE, 100);
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        break;
        
    case '2':
    analogWrite(ENABLE, 100);
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        break;
  }

}
