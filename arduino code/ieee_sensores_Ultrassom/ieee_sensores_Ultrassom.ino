
#include <ros.h>
#include <projeto_semear/Sonar_Infra_Placa_Sensores.h>
#include <Ultrasonic.h>
/* Pinouts Infravermelhos */
#define infra_FFL A0
#define infra_FBL A7
#define infra_BFL A1
#define infra_BBL A2
#define infra_SL A6

//Define os pinos para o trigger e echo
#define pino_trigger 4
#define pino_echo 5

projeto_semear::Sonar_Infra_Placa_Sensores infras;

Ultrasonic ultrasonic(pino_trigger, pino_echo);

ros::NodeHandle nh;

ros::Publisher pub_infras("/AMR/arduinoSensoresUltrassom", &infras ); //  Infra Info

void setup() {

  nh.getHardware()->setBaud(57600);
  nh.initNode();

  nh.advertise(pub_infras);

}

void loop() {

  infras.infraFFL = analogRead(infra_FFL);
  infras.infraFBL = analogRead(infra_FBL);
  infras.infraBFL = analogRead(infra_BFL);
  infras.infraBBL = analogRead(infra_BBL);
  infras.infraSL = analogRead(infra_SL);

  infras.sonar = ultrasonic.convert( ultrasonic.timing(), Ultrasonic::CM);

  pub_infras.publish(&infras);

  nh.spinOnce();
  delay(25);
}
