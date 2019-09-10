/* Esse código é responsável pelos seguintes sensores/atuadores:
 *
 * A) Motor DC da garra: 
 *    1 Encoder com canais A e B
 *    2 Inputs da Ponte H para controlar o sentido
 *    1 Enable para controlar PWM
 *    2 Fim de cursos
 *
 * B) Eletroima:
 *    1 Sinal de controle (para o BJT)
 *
 * C) 6 Sensores Pololu
 *    2 Para cada um dos 4 lados do robô
 *
 * D) 2 sensores Digitais de Infravermelho
 *    Para detectar os containers
 *
 * E) 1 Servo motor para controlar a rotação da garra
 *
 * Para controlar o arduino, é necessário utilizar comunicação serial.
 *
 * Envie um INT com o seguinte valor para conseguir a funcionalidade:
 *  256: Ligar o Eletroima
 *  257: Desliga Eletroima
 *  258: Liga controle do servo. Sequentemente, você precisa enviar a posição do servo
 *  259: Ligar o feedback do encoder, fim de curso e motor da garra
 *  260: Desligar o feedback do encoder, fim de curso e o motor da garra
 *  261: Ligar os sensores de linha
 *  262: Desligar sensores de linha
 *  263: Ligar os sensores de containers
 *  264: Desligar os sensores de containers
 *
 *   Caso o motor tenha sido ativado (enviando previamente 259), o PWM e o sentido podem ser controlados enviando valores entre [-255,255]
 *   Não recomenda-se ativar os sensores e o motor ao mesmo tempo
 *
 * O feedback dos motores são 2 Serial.print. Primeiro, envia-se um LONG INT com os ticks do motor e, posteriormente, um BOOL para avisar se algum fim de curso foi ativado.
 *
 * O feedback dos sensores são os 6 sensores da pololu seguido pelos 2 sensores digitais
 */

#include <QTRSensors.h>

#include "Arduino.h"
#include "pins_arduino.h"
#include <Servo.h>

// MOTOR GARRA
#define encoder1A 2 // D2
#define encoder1B 3 // D3
#define bitEncoder1A (1 << PCINT18)
#define bitEncoder1B (1 << PCINT19)

#define IN1 11
#define IN2 9
#define ENABLE 12

#define FIMCURSO1 10
#define FIMCURSO2 8
#define FIMCURSOBITS (1 << PCINT0) | (1 << PCINT2)

// MOTOR GARRA
volatile long int encoder_tick;
volatile int PWM = 0;

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

    // MOTOR GARRA
    pinMode(encoder1A, INPUT);
    pinMode(encoder1B, INPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENABLE, OUTPUT);

    attachInterrupt(digitalPinToInterrupt(encoder1A), encoder1A_cb, RISING); // D2
    attachInterrupt(digitalPinToInterrupt(encoder1B), encoder1B_cb, RISING); // D3

    delay(1000);
}

bool publish_encoder = false;

void loop()
{

    if (publish_encoder)
    {
        Serial.print(encoder_tick, DEC);
        Serial.print(PINB & FIMCURSOBITS, DEC);

        if (PWM > 0)
        {
            digitalWrite(IN1, HIGH);
            digitalWrite(IN2, LOW);
        }
        else if (PWM < 0)
        {
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, HIGH);
        }
        else
        {
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, LOW);
        }
        analogWrite(ENABLE, PWM);
    }
}

/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.

  char in range -128 to 127.
  uchar in range 0 255
*/
void serialEvent()
{

    char ch = Serial.read();

    if (-64 < ch || ch < 64)
    {
        PWM = round(ch * 4.04); // in range [-63, 63] -> x4 -> [-252, 252]
    }

    switch (ch)
    {

    case 64: // ligar o feedback do encoder, fim de curso e motor da garra
        publish_encoder = true;
        break;

    case 65: // Desliga feedback do encoder, fim de curso e motor da garra
        publish_encoder = false;
        PWM = 0;
        analogWrite(ENABLE, PWM);
        break;
    }
}
