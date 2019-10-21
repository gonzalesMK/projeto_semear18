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

#define IN1 12
#define IN2 9
#define ENABLE 11

#define FIMCURSO1 10 // esse avisa quando o eletroima foi pressionado
#define FIMCURSO2 8
#define FIMCURSOBITS ( (1 << PCINT0) | (1 << PCINT2) )

// SERVO
#define SERVO_PIN 6

// SENSORES Analógicos POLOLU de A0 até A7
#define CONTROL_PIN 5
#define IS_BLACK 90 
#define FL 0
#define FR 1
#define BL 3
#define BR 2
#define LF 5
#define LB 4
#define RF 6
#define RB 7

#define FL_BLACK 55
#define FR_BLACK 190
#define RF_BLACK 75
#define RB_BLACK 300
#define BL_BLACK 47
#define BR_BLACK 42
#define LF_BLACK 42 // bigger is green
#define LR_BLACK 45 // bigger is green

// Sensores Digitais
#define DIGI1 13
#define DIGI2 7

// ELETROIMA
#define ELETROIMA_PIN 4

// MOTOR GARRA
volatile long unsigned int encoder_tick =  2147483648;
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
uint8_t ordering[] = {FL, FR, BL, BR, LF, LB, RF, RB};
uint16_t is_black[] = {FL_BLACK, FR_BLACK, BL_BLACK, BR_BLACK, LF_BLACK, LR_BLACK, RF_BLACK, RB_BLACK}; 
// SERVO
Servo servo;

void setup()
{
  Serial.begin(115200);

  pinMode(CONTROL_PIN, OUTPUT);
  //digitalWrite(CONTROL_PIN, HIGH);    
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);
  qtr.setEmitterPin(CONTROL_PIN);
  qtr.emittersOff();
  qtr.setDimmingLevel(0);
  qtr.emittersOn();

  pinMode(DIGI1, INPUT_PULLUP);
  pinMode(DIGI2, INPUT_PULLUP);

  pinMode(FIMCURSO1, INPUT);
  pinMode(FIMCURSO2, INPUT);
 
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENABLE, OUTPUT);

  
  // MOTOR GARRA
  pinMode(encoder1A, INPUT);
  pinMode(encoder1B, INPUT);

  delay(3000);

  attachInterrupt(digitalPinToInterrupt(encoder1A), encoder1A_cb, RISING); // D2
  attachInterrupt(digitalPinToInterrupt(encoder1B), encoder1B_cb, RISING); // D3

  // SERVO
  servo.attach(SERVO_PIN);
  servo.write(65);
  // ELETROIMA
  pinMode(ELETROIMA_PIN, OUTPUT);
}

int servo_pose;
bool publish_encoder = false;
bool publish_sensors = false;
bool publish_containers_sensors = false;

unsigned long time;
void loop()
{
  time = millis();

  qtr.emittersOn();

  sensorValues[3] = analogRead(A3);
  sensorValues[3] = analogRead(A3);
  
  sensorValues[2] = analogRead(A2);
  sensorValues[2] = analogRead(A2);

  sensorValues[5] = analogRead(A5);
  sensorValues[5] = analogRead(A5);
  sensorValues[4] = analogRead(A4);
  sensorValues[4] = analogRead(A4);

  sensorValues[7] = analogRead(A7);
  sensorValues[7] = analogRead(A7);
  sensorValues[6] = analogRead(A6);
  sensorValues[6] = analogRead(A6);

  sensorValues[1] = analogRead(A1);
  sensorValues[1] = analogRead(A1);

  sensorValues[0] = analogRead(A0);
  sensorValues[0] = analogRead(A0);
  qtr.emittersOff();
  
  uint8_t alissonSensors = (uint8_t)digitalRead(DIGI1) + digitalRead(DIGI2) * 2;

  if (PWM > 0)
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
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
    analogWrite(ENABLE, 255);
  }
  
  byte sensorsValuesByte = 0;
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    sensorsValuesByte |= ((sensorValues[ordering[i]] > IS_BLACK) << i);
  }

  Serial.write(sensorsValuesByte);
  Serial.write(alissonSensors);
  Serial.write( PINB & FIMCURSOBITS );
  Serial.write( encoder_tick & 0xFF);
  Serial.write(((encoder_tick >> 8) & 0xFF));
  Serial.write(((encoder_tick >> 16) & 0xFF));
  Serial.write(((encoder_tick >> 24) & 0xFF));
  

  while (millis() - time < 5)
  {
  }
}

/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
void serialEvent()
{

  char ch = (char)Serial.read();

  if (-63 <= ch && ch <= 63)
  {
    PWM = ch * 4;
  }

  switch (ch)
  {
  case 64: // Liga Eletroima
    digitalWrite(ELETROIMA_PIN, HIGH);
    break;

  case 65: // Desliga Eletroima
    digitalWrite(ELETROIMA_PIN, LOW);
    break;

  case 66: // Liga controle do servo e pede por posição
    servo_pose = -1;

    while (servo_pose == -1)
    {
      if (Serial.available())
      {
        servo_pose = (uint8_t)Serial.read();
      }
    }

    servo.write(servo_pose);
    break;
  }
}
