#include <ros.h>
#include <projeto_semear/ArduinoRGB.h>
#include <projeto_semear/Infra_Placa_Sensores.h>
#include <std_msgs/Bool.h>

#include <Wire.h>
#include "Adafruit_TCS34725.h"

/* Pinouts Infravermelhos */
#define infra_FFR A1
#define infra_FBR A0
#define infra_BFR A2
#define infra_BBR A6
#define infra_SR A2

#define SDApin A4
#define SCLpin A5

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

projeto_semear::ArduinoRGB rgb;
projeto_semear::Infra_Placa_Sensores infras;

ros::NodeHandle nh;

ros::Publisher pub_rgb("/AMR/arduinoRGB", &rgb ); //  RGB info
ros::Publisher pub_infras("/AMR/arduinoSensoresRGBInfras", &infras ); //  Infra Info

void enable_callback(const std_msgs::Bool &msg ); // Enable Callback

ros::Subscriber<std_msgs::Bool> sub_enables("/AMR/enableRGB", &enable_callback); // Subscrive to enable topic

bool enable_rgb = false;

void setup() {

  nh.getHardware()->setBaud(9600);
  nh.initNode();

  nh.advertise(pub_infras);
  nh.advertise(pub_rgb);
  
  nh.subscribe(sub_enables);

  /* Set the RGB */
  tcs.begin();

}

void loop() {

  if ( !enable_rgb) {
    infras.infraFFR = analogRead(infra_FFR);
    infras.infraFBR = analogRead(infra_FBR);
    infras.infraBFR = analogRead(infra_BFR);
    infras.infraBBR = analogRead(infra_BBR);
    infras.infraSR = analogRead(infra_SR);

    pub_infras.publish(&infras);

    rgb.red = 0;
    rgb.green = 0;
    rgb.blue = 0 ;
    pub_rgb.publish(&rgb);
  
  }

  if ( enable_rgb ) {
    uint16_t clear, red, green, blue;

    tcs.getRawData(&red, &green, &blue, &clear);

    rgb.red = red;
    rgb.green = green;
    rgb.blue = blue;

    pub_rgb.publish(&rgb);
  }

  nh.spinOnce();
  delay(50);
}

void enable_callback( const std_msgs::Bool &msg )
{
  enable_rgb = msg.data;
}        
