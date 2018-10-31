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
  pinMode(infraFD, INPUT);
  pinMode(infraFL, INPUT);
  pinMode(infraRR, INPUT);
  pinMode(infraRL, INPUT);
  pinMode(infraSE, INPUT);
  pinMode(infraSD, INPUT);
}
double teste;
void loop() {

    infras.infraFR = analogRead(infra_FD);
    infras.infraFL = analogRead(infra_FL);
    infras.infraBR = analogRead(infra_RR);
    infras.infraBL = analogRead(infra_RL);
    infras.infraBL = analogRead(infra_SE);
    infras.infraBL = analogRead(infra_SD);

    pub_infras.publish(infras);
    delay(10);
}

void enable_callback( const projeto_semear::Enable_Placa_Elevadores &msg )
{
}
