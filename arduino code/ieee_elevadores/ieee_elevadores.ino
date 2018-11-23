// Data: 26/10/2018
#include <ros.h>
#include <ros/time.h>
#include <projeto_semear/Vel_Elevadores.h>
#include <projeto_semear/Infra_Placa_Elevadores.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <PinChangeInterrupt.h>
#include <PinChangeInterruptBoards.h>
#include <PinChangeInterruptSettings.h>
#include <PinChangeInterruptPins.h>

#include <Servo.h>

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
#define motorCremalheiraHorizontal_A    35  // RINA1 servo 2
#define motorCremalheiraHorizontal_B    37  // RINA2 
#define motorCremalheiraHorizontal_enable 4
#define motorCremalheiraVertical_A    39 // RINA3 servo 1
#define motorCremalheiraVertical_B    41 // RINA4
#define motorCremalheiraVertical_enable 5

/* Pinout dos Encoders: */
#define encoderCremalheiraVertical_chA 2
#define encoderCremalheiraVertical_chB 3
#define encoderCremalheiraHorizontal_chA 18
#define encoderCremalheiraHorizontal_chB 19

/* Pinouts Relés */
#define eletroima 51

/* Pinout Fim de cursos */
#define cremalheira_vertical_baixo 43 // INA1 -- Deve estar na garra
#define cremalheira_vertical_alto 45  // INA2
#define cremalheira_horizontal_frente 47 // INA3
#define cremalheira_horizontal_tras 49 // INA4

/* Pinout Servos */
#define servo 13

/* Constantes: */
#define dt           0.05    // Loop time [s]    
#define pi           3.14159265
#define count_num    6600   // count number of encoder * 4 

const double AnglePerCount = 2 * pi / 6600;

/* Motor related global variables: */
volatile long tick_V = 0.0,
              tick_H = 0.0;

long _Prev_V_Ticks = 0.0,
     _Prev_H_Ticks = 0.0;

volatile long int wMotor[2] = {0.0, 0.0};

enum motor {
  V = 0,
  H = 1,
} ;

/* Eletroima Global Bariable */
volatile bool enable_eletroima = false;

/* Pose Global Variable */
Servo s;
int servo_pose = -1;

/* Set up the ros node, publishers and subscribers */
ros::NodeHandle nh;

projeto_semear::Vel_Elevadores output_displacement;
projeto_semear::Vel_Elevadores input_vel;
projeto_semear::Infra_Placa_Elevadores infras;
std_msgs::Bool enables;

/* Publishers */
ros::Publisher pub_output_displacement("/AMR/arduinoElevadoresOutputDisplacement", &output_displacement); //  Publish real velocity in Rad/s

/* Callbacks for ROS */
void cmd_vel_callback( const projeto_semear::Vel_Elevadores &vel ); // Input Vel Callback:
void enable_eletroima_callback( const std_msgs::Bool &msg ); // Enable Callback
void servo_pose_callback(const std_msgs::Int16 &msg); // Receive the servo position

/* Subscribers */
ros::Subscriber<projeto_semear::Vel_Elevadores> sub_cmd_vel("/AMR/arduinoElevadoresInputVel", &cmd_vel_callback); // Subscribe to input velocity
ros::Subscriber<std_msgs::Bool> sub_enable_eletroima("/AMR/activateEletroima", &enable_eletroima_callback); // Subscribe to enable topic
ros::Subscriber<std_msgs::Int16> sub_servo_pose("AMR/servo", &servo_pose_callback);

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
  nh.getHardware()->setBaud(9600);

  nh.initNode();

  nh.advertise(pub_output_displacement);

  nh.subscribe(sub_cmd_vel);
  nh.subscribe(sub_enable_eletroima);
  nh.subscribe(sub_servo_pose);

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
  pinMode( eletroima, OUTPUT);

  /* Set the Fim de Curso (microSwitch) pins */
  pinMode(cremalheira_vertical_baixo    , INPUT_PULLUP);
  pinMode(cremalheira_vertical_alto     , INPUT_PULLUP);
  pinMode(cremalheira_horizontal_frente , INPUT_PULLUP);
  pinMode(cremalheira_horizontal_tras   , INPUT_PULLUP);

  /* Servo motor */
  s.attach(servo);
}

void loop()
{
  unsigned long start_time = millis();

  digitalWrite( eletroima, enable_eletroima);

  if ( servo_pose != -1 ) {
    s.write(servo_pose);
  }

  // encoder displacement of each wheel
  output_displacement.CremalheiraVertical   = tick_V;
  output_displacement.CremalheiraHorizontal = tick_H;

  pub_output_displacement.publish(&output_displacement);

  for ( int i = 0; i < 2 ; i++) {
    if ( wMotor[i] > 255 )wMotor[i] = 255;
    else if ( wMotor[i] < - 255)wMotor[i] = -255;
  }

  analogWrite(motorCremalheiraVertical_enable, abs(wMotor[V]));
  if (wMotor[V] < 0)
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
  if (wMotor[H] > 0)
  {
    digitalWrite(motorCremalheiraHorizontal_B, HIGH);
    digitalWrite(motorCremalheiraHorizontal_A, LOW);
  }
  else
  {
    digitalWrite(motorCremalheiraHorizontal_A, HIGH);
    digitalWrite(motorCremalheiraHorizontal_B, LOW);
  }
  delay(50);
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
    tick_H--;
  else
    tick_H++;
}

void encoder_H_chB_cb()
{
  int chA = digitalRead(encoderCremalheiraHorizontal_chA);
  int chB = digitalRead(encoderCremalheiraHorizontal_chB);

  if (chB == chA)
    tick_H--;
  else
    tick_H++;
}

void cmd_vel_callback( const projeto_semear::Vel_Elevadores &vel )
{
  wMotor[V] = vel.CremalheiraVertical;
  wMotor[H] = vel.CremalheiraHorizontal;
}

void enable_eletroima_callback( const std_msgs::Bool &msg )
{
  enable_eletroima  = msg.data;
}

void servo_pose_callback(const std_msgs::Int16 &msg)
{
  servo_pose = msg.data;
}

