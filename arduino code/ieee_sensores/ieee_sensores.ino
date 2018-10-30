#include <ros.h>
#include <ros/time.h>
#include <projeto_semear/ArduinoRGB.h>
#include <projeto_semear/Infra_Placa_Sensores.h>
#include <projeto_semear/Enable_Placa_Sensores.h>

#include <Wire.h>
#include "Adafruit_TCS34725.h"

/* Pinouts Infravermelhos */
#define infraFD A0
#define infraFL A1
#define infraRR A2
#define infraRL A3
#define infraSE A6
#define infraSD A7

#define SDApin A4
#define SCLpin A5

projeto_semear::ArduinoRGB rgb;
projeto_semear::Infra_Placa_Sensores infras;

ros::Publisher pub_rgb("/AMR/arduinoSensoresRGB", &rgb ); //  RGB info
ros::Publisher pub_infras("/AMR/arduinoSensoresInfras", &infras ); //  Infra Info

void enable_callback(const projeto_semear::Enable_Placa_Sensores &msg ); // Enable Callback

ros::Subscriber<projeto_semear::Enable_Placa_Sensores> sub_enables("/AMR/enableElevadores", &enable_callback); // Subscrive to enable topic

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

bool enable_rgb = false;
bool enable_infra = false;

void setup() {
  
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println( analogRead(infraFD) );

  delay(100);
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

