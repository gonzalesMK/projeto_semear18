// Data: 26/10/2018
#include <ros.h>
#include <ros/time.h>
#include <projeto_semear/Vel_Elevadores.h>
#include <projeto_semear/ArduinoRGB.h>
#include <projeto_semear/Infra_Placa_Elevadores.h>
#include <projeto_semear/Enable_Placa_Elevadores.h>


#include <PinChangeInterrupt.h>
#include <PinChangeInterruptBoards.h>
#include <PinChangeInterruptSettings.h>
#include <PinChangeInterruptPins.h>

#include <Wire.h>
#include "Adafruit_TCS34725.h"

#include <PID_v1.h>

/* A placa dos elevadores possui os seguintes conponentes:
    4 infras
    2 eletroimas (relés)
    1 RGB
    2 Servos de 6 entradas
    1 Servo de 3 entradas,
    5 fim de cursos

    FALTA PROGRAMAR OS FINS DE CURSOS
*/

/* Pinout dos Motores: */
#define motorCremalheiraVertical_A    7
#define motorCremalheiraVertical_B    5
#define motorCremalheiraVertical_enable 12
#define motorCremalheiraHorizontal_A    9
#define motorCremalheiraHorizontal_B    8
#define motorCremalheiraHorizontal_enable 11

/* Pinout dos Encoders: */
#define encoderCremalheiraVertical_chA 18
#define encoderCremalheiraVertical_chB 19
#define encoderCremalheiraHorizontal_chA 21
#define encoderCremalheiraHorizontal_chB 51

/* Pinouts Infravermelhos */
#define infra_FR A1
#define infra_FL A2
#define infra_BR A3
#define infra_BL A4

/* Pinouts RGB */
#define SDApin 20
#define SCLpin 21

/* Pinouts Relés */
#define releL 15
#define releR 16

/* Pinout Fim de cursos */
#define cremalheira_vertical_baixo 16
#define cremalheira_vertical_alto 17
#define cremalheira_horizontal_frente 18
#define cremalheira_horizontal_tras 19
#define garra 20

/* Pinout Servo */
#define servo 15 // PWM

/* Constantes: */
#define dt           0.05    // Loop time [s]    
#define pi           3.14159265
#define count_num    6600   // count number of encoder * 4 

const double AnglePerCount = 2 * pi / 6600;

/* Motor related global variables: */
volatile long tick_V= 0.0,
              tick_H = 0.0;

long _Prev_V_Ticks = 0.0,
     _Prev_H_Ticks = 0.0;

volatile int wMotor[2] = {0.0, 0.0};

enum motor {
  V = 0,
  H = 1,
} ;

/* Enable related global variable */
bool enable_motor = false;
bool enable_servo = false;
bool enable_infra = false;
bool enable_rele = false;
bool enable_rgb = false;

/* RGB */
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

/* Servo
   A posição do servo em relação ao pwm é:
    PREENCHER !!
*/
uint32_t servo_pwm = 0;

// Set up the ros node, publishers and subscribers
ros::NodeHandle nh;

projeto_semear::Vel_Elevadores output_vel;
projeto_semear::Vel_Elevadores input_vel;
projeto_semear::Enable_Placa_Elevadores enables;
projeto_semear::ArduinoRGB rgb;
projeto_semear::Infra_Placa_Elevadores infras;

/* Publishers */
ros::Publisher pub_output_vel("/AMR/arduinoElevadoresOutputVel", &output_vel); //  Publish real velocity in Rad/s
ros::Publisher pub_rgb("/AMR/arduinoElevadoresRGB", &rgb ); //  RGB info
ros::Publisher pub_infras("/AMR/arduinoElevadoresInfras", &infras ); //  Infra Info

/* Callbacks for ROS */
void cmd_vel_callback( const projeto_semear::Vel_Elevadores &vel ); // Input Vel Callback:
void enable_callback( const projeto_semear::Enable_Placa_Elevadores &msg ); // Enable Callback

/* Subscribers */
ros::Subscriber<projeto_semear::Vel_Elevadores> sub_cmd_vel("/AMR/arduinoElevadoresInputVel", &cmd_vel_callback); // Subscribe to input velocity
ros::Subscriber<projeto_semear::Enable_Placa_Elevadores> sub_enables("/AMR/cmdVel", &enable_callback); // Subscrive to enable topic

/* Callback for interruptions from encoder */
void encoder_V_chA_cb();
void encoder_V_chB_cb();
void encoder_H_chA_cb();
void encoder_H_chB_cb();

/* Callback for interruptions from Fim de Cursos (microSwitch) */
void cremalheira_vertical_baixo_cb();
void cremalheira_vertical_alto_cb();
void cremalheira_horizontal_frente_cb();
void cremalheira_horizontal_tras_cb();
void garra_cb();

void setup()
{
  // set the baud rate
  nh.getHardware()->setBaud(57600);

  nh.initNode();

  nh.advertise(pub_output_vel);

  nh.subscribe(sub_cmd_vel);
  nh.subscribe(sub_enables);

  /* Set the motor pins: */
  pinMode(motorCremalheiraVertical_A, OUTPUT);
  pinMode(motorCremalheiraVertical_B, OUTPUT);
  pinMode(motorCremalheiraVertical_enable, OUTPUT);
  pinMode(motorCremalheiraHorizontal_A, OUTPUT);
  pinMode(motorCremalheiraHorizontal_B, OUTPUT);
  pinMode(motorCremalheiraHorizontal_enable, OUTPUT);

  /* Set the encoder Pins */
  pinMode(encoderCremalheiraVertical_chA, INPUT_PULLUP);
  pinMode(encoderCremalheiraHorizontal_chA, INPUT_PULLUP);
  pinMode(encoderCremalheiraVertical_chB, INPUT_PULLUP);
  pinMode(encoderCremalheiraHorizontal_chB, INPUT_PULLUP);

  /* Set encoder interruptions */
  attachInterrupt(digitalPinToInterrupt(encoderCremalheiraVertical_chA), encoder_V_chA_cb, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderCremalheiraHorizontal_chA), encoder_H_chA_cb, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderCremalheiraVertical_chB), encoder_V_chB_cb, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderCremalheiraHorizontal_chB), encoder_H_chB_cb, CHANGE);

  /* Set the Relés (relay) pins */
  pinMode( releL, OUTPUT);
  pinMode( releR, OUTPUT);

  /* Set the Fim de Curso (microSwitch) pins */
  pinMode(cremalheira_vertical_baixo    , INPUT_PULLUP);
  pinMode(cremalheira_vertical_alto     , INPUT_PULLUP);
  pinMode(cremalheira_horizontal_frente , INPUT_PULLUP);
  pinMode(cremalheira_horizontal_tras   , INPUT_PULLUP);
  pinMode(garra                         , INPUT_PULLUP);

  /* Set the Servo (slave) pins*/
  pinMode(servo, OUTPUT);

  /* Set the RGB */
  if (!tcs.begin())  while (10);
}

void loop()
{
  unsigned long start_time = millis();

  if (enable_rgb) {
    uint16_t clear, red, green, blue;

    tcs.getRawData(&red, &green, &blue, &clear);

    /* send a MESSAGE TO ROS WITH RGB !!!!!!!!! */

  }

  if ( enable_infra) {
    /* send a MESSAGE TO ROS WITH INFRAS !!!!!!!!*/
    infras.infraFR = analogRead(infra_FR);
    infras.infraFL = analogRead(infra_FL);
    infras.infraBR = analogRead(infra_BR);
    infras.infraBL = analogRead(infra_BL);
  }

  if ( enable_rele) {
    digitalWrite( releR, HIGH);
    digitalWrite( releL, HIGH);
  } else {
    digitalWrite( releR, LOW);
    digitalWrite( releL, LOW);
  }

  if ( enable_servo) {
    digitalWrite( servo, servo_pwm);
  }

  if ( enable_motor) {
    // extract the wheel velocities from the tick signals count
    int deltaV = tick_V - _Prev_V_Ticks;
    int deltaH = tick_H - _Prev_H_Ticks;

    // angular velocity of each wheel
    double omega_V = (deltaV * AnglePerCount) / dt;
    double omega_H = (deltaH * AnglePerCount) / dt;

    // Publish Real Velocity
    output_vel.CremalheiraVertical = omega_V; // /(2*pi);
    output_vel.CremalheiraHorizontal = omega_H; // /(2*pi);
    pub_output_vel.publish(&output_vel);

    int i = 0;
    for (i = 0; i < 2 ; i++) {
      if ( wMotor[i] > 255 )wMotor[i] = 255;
      else if ( wMotor[i] < - 255)wMotor[i] = -255;
    }

    analogWrite(motorCremalheiraVertical_enable, abs(wMotor[V]));
    if (wMotor[V] > 0)
    {
      digitalWrite(motorCremalheiraVertical_B, HIGH);
      digitalWrite(motorCremalheiraVertical_A, LOW);
    }
    else
    {
      digitalWrite(motorCremalheiraVertical_A, HIGH);
      digitalWrite(motorCremalheiraVertical_B, LOW);
    }

    analogWrite(motorCremalheiraHorizontal_enable, abs(wMotor[H]));
    if (wMotor[H] < 0)
    {
      digitalWrite(motorCremalheiraHorizontal_B, HIGH);
      digitalWrite(motorCremalheiraHorizontal_A, LOW);
    }
    else
    {
      digitalWrite(motorCremalheiraHorizontal_A, HIGH);
      digitalWrite(motorCremalheiraHorizontal_B, LOW);
    }

    /* Setting encoder variables */
    _Prev_V_Ticks = tick_V;
    _Prev_H_Ticks = tick_H;
  }

  delay( dt * 1000 - (millis() - start_time) );
  nh.spinOnce();

}

void encoder_V_chA_cb()
{
  int chB = digitalRead(encoderCremalheiraVertical_chB);
  int chA = digitalRead(encoderCremalheiraVertical_chA);

  if (chB != chA)
    tick_V++;
  else
    tick_V--;
}

void encoder_V_chB_cb() {
  int chB = digitalRead(encoderCremalheiraVertical_chB);
  int chA = digitalRead(encoderCremalheiraVertical_chA);

  if (chB == chA)
    tick_V++;
  else
    tick_V--;

}

void encoder_H_chA_cb()
{
  int chA = digitalRead(encoderCremalheiraHorizontal_chA);
  int chB = digitalRead(encoderCremalheiraHorizontal_chB);

  if (chB != chA)
    tick_H++;
  else
    tick_H--;
}

void encoder_H_chB_cb()
{
  int chA = digitalRead(encoderCremalheiraHorizontal_chA);
  int chB = digitalRead(encoderCremalheiraHorizontal_chB);

  if (chB == chA)
    tick_H++;
  else
    tick_H--;
}

void cmd_vel_callback( const projeto_semear::Vel_Elevadores &vel )
{
  wMotor[V] += (int) vel.CremalheiraVertical;
  wMotor[H] += (int) vel.CremalheiraHorizontal;
}

void enable_callback( const projeto_semear::Enable_Placa_Elevadores &msg )
{
  enable_motor = msg.enable_motor;
  enable_servo = msg.enable_servo;
  enable_infra = msg.enable_infra;
  enable_rele  = msg.enable_rele;
  enable_rgb   = msg.enable_rgb;

  servo_pwm = msg.servo_pwm;
}

