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
 *  259: Ligar o feedback do encoder, fim de curso e motor da garra. 
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

// SERVO
#define SERVO_PIN 6

// SENSORES Analógicos POLOLU de A0 até A7
#define CONTROL_PIN 5
#define IS_BLACK 500
// Sensores Digitais
#define DIGI1 13
#define DIGI2 7

// ELETROIMA
#define ELETROIMA_PIN 4

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

// SENSORES POLOLU
QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

// SERVO
Servo servo;

void setup()
{
  Serial.begin(115200);

  // POLOLU
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);
  qtr.setEmitterPin(CONTROL_PIN);

  pinMode(DIGI1, INPUT);
  pinMode(DIGI2, INPUT);

  // MOTOR GARRA
  pinMode(encoder1A, INPUT);
  pinMode(encoder1B, INPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENABLE, OUTPUT);
  
  // SERVO
  servo.attach(SERVO_PIN);

  // ELETROIMA
  pinMode(ELETROIMA_PIN, OUTPUT);
  
  attachInterrupt(digitalPinToInterrupt(encoder1A), encoder1A_cb, RISING); // D2
  attachInterrupt(digitalPinToInterrupt(encoder1B), encoder1B_cb, RISING); // D3

  
  delay(1000);
}

int servo_pose;
bool publish_encoder = false;
bool publish_sensors = false;
bool publish_containers_sensors = false;

void loop()
{


  if (publish_sensors)
  {
    qtr.readLineBlack(sensorValues);

    byte sensorsValuesByte = 0;
    for (uint8_t i = 0; i < SensorCount; i++)
    {
      sensorsValuesByte |= ((sensorValues[i] > IS_BLACK) << i);
    }
    Serial.print(sensorsValuesByte);
    Serial.println();
  }

  if (publish_containers_sensors)
  {
    
    Serial.print(digitalRead(DIGI1));
    Serial.print(digitalRead(DIGI2));
    Serial.println();
  }
   
   if (publish_encoder)
  {
    Serial.print("Fim de Cursos: ");
    Serial.print((bool)PINB & FIMCURSOBITS, DEC);
    
    Serial.print(" Encoder Ticks: ");
    Serial.print(encoder_tick, DEC);
    
    
    if (PWM > 0)
    {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      Serial.print( "Motor going Positive");
      analogWrite(ENABLE, abs(PWM));
    }
    else if (PWM < 0)
    {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      analogWrite(ENABLE, abs(PWM));
    }
    else
    {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
    }
    
    Serial.println();
  }
  
  delay(500);
}

/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
void serialEvent()
{

  char ch = Serial.read();
  if( ch == 10){
    return;
  }
  Serial.print("Read: ");
  Serial.print(ch, DEC);
  
  
  if (-63 <= ch && ch <= 63)
  {
    PWM = ch*4;
    Serial.print( "New PWM: ");
    Serial.print(PWM);
    Serial.println();
  }

  switch (ch)
  {
  case 64: // Liga Eletroima
    digitalWrite(ELETROIMA_PIN, HIGH);
    break;

  case 65: // Desliga Eletroima
    digitalWrite(ELETROIMA_PIN, LOW);
    break;

  case 66: // Liga controle do servo e pede por posição 'B'
    servo_pose = -1;
    
    Serial.print("set the servo pose: ");
    
    while (servo_pose == -1)
    {
      if (Serial.available())
      {
        servo_pose = (uint8_t) Serial.read();
      }
      if (servo_pose == 10) servo_pose = -1;
    }
    
    Serial.print(" Servo Pose: ");
    Serial.print( servo_pose);
    Serial.println();

    servo.write(servo_pose);
    break;

  case 67: // ligar o feedback do encoder, fim de curso e motor da garra 'C'
    publish_encoder = true;
    break;

  case 68: // Desliga feedback do encoder, fim de curso e motor da garra 'D'
    publish_encoder = false;
    PWM = 0;
    //analogWrite(ENABLE, PWM);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    break;

  case 69: // Liga sensores de linha 'E'
    publish_sensors = true;
    break;

  case 70: // Desliga sensores de linha
    publish_sensors = false;
    break;

  case 71:
    publish_containers_sensors = true;
    break;

  case 72:
    publish_containers_sensors = false;
    break;
  }
  Serial.println();
}
