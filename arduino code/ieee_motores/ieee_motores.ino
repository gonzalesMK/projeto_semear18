// Data: 18/10/2017
#include <ros.h>
#include <ros/time.h>
#include <projeto_semear/Vel.h>

#include <PinChangeInterrupt.h>
#include <PinChangeInterruptBoards.h>
#include <PinChangeInterruptSettings.h>
#include <PinChangeInterruptPins.h>

/* Parametros dos Motores: */
#define motorFR_A    37  // OUT 1 e OUT2
#define motorFR_B    39
#define motorFR_enable 4

#define motorFL_A    36 // OUT5 e OUT6
#define motorFL_B    38
#define motorFL_enable 6

#define motorBR_A    40 // OUT7 e OUT8
#define motorBR_B    42
#define motorBR_enable 7

#define motorBL_A    41 // OUT 3 e OUT4
#define motorBL_B    43
#define motorBL_enable 5

/* Parametros dos Encoders: */
#define encoderFR_chA 18 //  Native Interruption Pin
#define encoderFR_chB 19 //  Native Interruption Pin 
#define encoderBL_chA 20 //  Native Interruption Pin
#define encoderBL_chB 51
#define encoderBR_chA 3  //  Native Interruption Pin
#define encoderBR_chB 2  //  Native Interruption Pin
#define encoderFL_chA 21 //  Native Interruption Pin
#define encoderFL_chB 50

/* Constantes: */
#define dt           0.05    // Loop time [s]    
#define pi           3.14159265
#define count_num    6600   // count number of encoder * 4 

const double AnglePerCount = 2 * pi / count_num;

// Global variables:
projeto_semear::Vel output_vel;
projeto_semear::Vel output_pwm;
projeto_semear::Vel input_vel;

volatile long tick_FR = 0.0,
              tick_FL = 0.0,
              tick_BR = 0.0,
              tick_BL = 0.0;

long _Prev_FR_Ticks = 0.0,
     _Prev_FL_Ticks = 0.0,
     _Prev_BR_Ticks = 0.0,
     _Prev_BL_Ticks = 0.0;

volatile int wMotor[4] = {0.0, 0.0, 0.0, 0.0};

enum motor {
  FR = 0,
  FL = 1,
  BR = 2,
  BL = 3
} ;

// Callback:
void cmd_vel_callback( const projeto_semear::Vel &vel );

// Interruptions callback
void encoder_FR_chA_cb();
void encoder_FR_chB_cb();
void encoder_FL_chA_cb();
void encoder_FL_chB_cb();
void encoder_BR_chA_cb();
void encoder_BR_chB_cb();
void encoder_BL_chA_cb();
void encoder_BL_chB_cb();

// Set up the ros node, publishers and subscribers
ros::NodeHandle nh;

ros::Publisher pub_output_vel("/AMR/arduinoVel", &output_vel);
ros::Publisher pub_output_pwm("/AMR/arduinoPWM", &output_pwm);
ros::Subscriber<projeto_semear::Vel> sub_cmd_vel("/AMR/InputVelBase", &cmd_vel_callback);

void setup()
{
  // set the baud rate
  nh.getHardware()->setBaud(57600);

  nh.initNode();

  nh.advertise(pub_output_vel);
  // DEBUG
  nh.advertise(pub_output_pwm);
  nh.subscribe(sub_cmd_vel);

  // Set the motor pins:
  pinMode(motorFR_A, OUTPUT);
  pinMode(motorFR_B, OUTPUT);
  pinMode(motorFR_enable, OUTPUT);

  pinMode(motorFL_A, OUTPUT);
  pinMode(motorFL_B, OUTPUT);
  pinMode(motorFL_enable, OUTPUT);

  pinMode(motorBR_A, OUTPUT);
  pinMode(motorBR_B, OUTPUT);
  pinMode(motorBR_enable, OUTPUT);

  pinMode(motorBL_A, OUTPUT);
  pinMode(motorBL_B, OUTPUT);
  pinMode(motorBL_enable, OUTPUT);

  // Set the encoder Pins
  pinMode( encoderFR_chA, INPUT_PULLUP);
  pinMode( encoderFL_chA, INPUT_PULLUP);
  pinMode( encoderBR_chA, INPUT_PULLUP);
  pinMode( encoderBL_chA, INPUT_PULLUP);
  pinMode( encoderFR_chB, INPUT_PULLUP);
  pinMode( encoderFL_chB, INPUT_PULLUP);
  pinMode( encoderBR_chB, INPUT_PULLUP);
  pinMode( encoderBL_chB, INPUT_PULLUP);

  // Set encoder pins
  attachInterrupt(digitalPinToInterrupt(encoderFR_chA), encoder_FR_chA_cb, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderFL_chA), encoder_FL_chA_cb, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderBR_chA), encoder_BR_chA_cb, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderBL_chA), encoder_BL_chA_cb, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderFR_chB), encoder_FR_chB_cb, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderBR_chB), encoder_BR_chB_cb, CHANGE);

  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(encoderFL_chB), encoder_FL_chB_cb, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(encoderBL_chB), encoder_BL_chB_cb, CHANGE);
}

void loop()
{
  unsigned long start_time = millis();

  // extract the wheel velocities from the tick signals count
  int deltaFR = tick_FR - _Prev_FR_Ticks;
  int deltaFL = tick_FL - _Prev_FL_Ticks;
  int deltaBR = tick_BR - _Prev_BR_Ticks;
  int deltaBL = tick_BL - _Prev_BL_Ticks;

  // angular velocity of each wheel
  double omega_FR = (deltaFR * AnglePerCount) / dt;
  double omega_FL = (deltaFL * AnglePerCount) / dt;
  double omega_BR = (deltaBR * AnglePerCount) / dt;
  double omega_BL = (deltaBL * AnglePerCount) / dt;

  output_vel.wFR = omega_FR;
  output_vel.wFL = omega_FL;
  output_vel.wBR = omega_BR;
  output_vel.wBL = omega_BL;

  output_pwm.wFR = wMotor[FR];
  output_pwm.wFL = wMotor[FL];
  output_pwm.wBR = wMotor[BR];
  output_pwm.wBL = wMotor[BL];
  pub_output_vel.publish(&output_vel);
  pub_output_pwm.publish(&output_pwm);

  int i = 0;
  for (i = 0; i < 4 ; i++) {

    if ( wMotor[i] > 255 ) {

      wMotor[i] = 255;

    } else if ( wMotor[i] < - 255) {

      wMotor[i] = -255;

    }
  }

  analogWrite(motorFR_enable, abs(wMotor[FR]));
  if (wMotor[FR] > 0)
  {
    digitalWrite(motorFR_B, HIGH);
    digitalWrite(motorFR_A, LOW);
  }
  else
  {
    digitalWrite(motorFR_A, HIGH);
    digitalWrite(motorFR_B, LOW);
  }

  analogWrite(motorBL_enable, abs(wMotor[BL]));
  if (wMotor[BL] < 0)
  {
    digitalWrite(motorBL_B, HIGH);
    digitalWrite(motorBL_A, LOW);
  }
  else
  {
    digitalWrite(motorBL_A, HIGH);
    digitalWrite(motorBL_B, LOW);
  }

  analogWrite(motorFL_enable, abs(wMotor[FL]));
  if (wMotor[FL] < 0)
  {
    digitalWrite(motorFL_B, HIGH);
    digitalWrite(motorFL_A, LOW);
  }
  else
  {
    digitalWrite(motorFL_A, HIGH);
    digitalWrite(motorFL_B, LOW);
  }

  analogWrite(motorBR_enable, abs(wMotor[BR]));
  if (wMotor[BR] < 0)
  {
    digitalWrite(motorBR_B, HIGH);
    digitalWrite(motorBR_A, LOW);
  }
  else
  {
    digitalWrite(motorBR_A, HIGH);
    digitalWrite(motorBR_B, LOW);
  }

  /* Setting encoder variables */
  _Prev_FR_Ticks = tick_FR;
  _Prev_FL_Ticks = tick_FL;
  _Prev_BR_Ticks = tick_BR;
  _Prev_BL_Ticks = tick_BL;

  delay( dt * 1000 - (millis() - start_time) );

  nh.spinOnce();

}

void encoder_FR_chA_cb()
{
  int chB = digitalRead(encoderFR_chB);
  int chA = digitalRead(encoderFR_chA);

  if (chB != chA)
    tick_FR++;
  else
    tick_FR--;
}

void encoder_FR_chB_cb() {
  int chB = digitalRead(encoderFR_chB);
  int chA = digitalRead(encoderFR_chA);

  if (chB == chA)
    tick_FR++;
  else
    tick_FR--;

}

void encoder_FL_chA_cb()
{
  int chA = digitalRead(encoderFL_chA);
  int chB = digitalRead(encoderFL_chB);

  if (chB != chA)
    tick_FL++;
  else
    tick_FL--;
}

void encoder_FL_chB_cb() {
  int chA = digitalRead(encoderFL_chA);
  int chB = digitalRead(encoderFL_chB);

  if (chB == chA)
    tick_FL++;
  else
    tick_FL--;
}

void encoder_BR_chA_cb()
{
  int chB = digitalRead(encoderBR_chB);
  int chA = digitalRead(encoderBR_chA);

  if (chB != chA)
    tick_BR--;
  else
    tick_BR++;
}

void encoder_BR_chB_cb() {
  int chB = digitalRead(encoderBR_chB);
  int chA = digitalRead(encoderBR_chA);

  if (chB == chA)
    tick_BR--;
  else
    tick_BR++;
}

void encoder_BL_chA_cb()
{
  int chB = digitalRead(encoderBL_chB);
  int chA = digitalRead(encoderBL_chA);

  if (chB != chA)
    tick_BL++;
  else
    tick_BL--;
}

void encoder_BL_chB_cb(void) {
  int chB = digitalRead(encoderBL_chB);
  int chA = digitalRead(encoderBL_chA);

  if (chB == chA)
    tick_BL++;
  else
    tick_BL--;
}


void cmd_vel_callback( const projeto_semear::Vel &vel )
{
  wMotor[BL] += vel.wBL;
  wMotor[BR] += vel.wBR;
  wMotor[FL] += vel.wFL;
  wMotor[FR] += vel.wFR;
}



