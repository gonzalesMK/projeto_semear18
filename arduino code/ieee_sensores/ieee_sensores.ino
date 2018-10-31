#include <ros.h>
#include <ros/time.h>
#include <projeto_semear/ArduinoRGB.h>
#include <projeto_semear/Infra_Placa_Sensores.h>
#include <projeto_semear/Enable_Placa_Sensores.h>

#include <Wire.h>
#include "Adafruit_TCS34725.h"

/* Pinouts Infravermelhos */
#define infra_FR A0
#define infra_FL A1
#define infra_BR A2
#define infra_BL A3
#define infra_SR A6
#define infra_SL A7

#define SDApin A4
#define SCLpin A5

projeto_semear::ArduinoRGB rgb;
projeto_semear::Infra_Placa_Sensores infras;

ros::NodeHandle nh;
ros::Publisher pub_rgb("/AMR/arduinoSensoresRGB", &rgb ); //  RGB info
ros::Publisher pub_infras("/AMR/arduinoSensoresInfras", &infras ); //  Infra Info

void enable_callback(const projeto_semear::Enable_Placa_Sensores &msg ); // Enable Callback

ros::Subscriber<projeto_semear::Enable_Placa_Sensores> sub_enables("/AMR/enableElevadores", &enable_callback); // Subscrive to enable topic

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

bool enable_rgb = false;

void setup() {

  nh.getHardware()->setBaud(57600);
  nh.initNode();
  
  nh.advertise(pub_infras);
  nh.advertise(pub_rgb);
  nh.subscribe(sub_enables);
  
  pinMode(infra_FR, INPUT);
  pinMode(infra_FL, INPUT);
  pinMode(infra_BR, INPUT);
  pinMode(infra_BL, INPUT);
  pinMode(infra_SR, INPUT);
  pinMode(infra_SL, INPUT);
}

void loop() {

    infras.infraFR = analogRead(infra_FR);
    infras.infraFL = analogRead(infra_FL);
    infras.infraBR = analogRead(infra_BR);
    infras.infraBL = analogRead(infra_BL);
    infras.infraSR = analogRead(infra_SR);
    infras.infraSL = analogRead(infra_SL);

    pub_infras.publish(&infras);
    delay(10);
}

void enable_callback( const projeto_semear::Enable_Placa_Sensores &msg )
{
  enable_rgb = msg.enable_rgb;
}
