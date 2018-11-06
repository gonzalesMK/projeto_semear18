
#include <ros.h>
#include <projeto_semear/Sonar_Infra_Placa_Sensores.h>
#include <Ultrasonic.h>
/* Pinouts Infravermelhos */
#define infra_FR A6
#define infra_FL A1 // está com problemas
#define infra_BR A2
#define infra_BL A3
#define infra_SL A7

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

  infras.infraFR = analogRead(infra_FR);
  infras.infraFL = analogRead(infra_FL);
  infras.infraBR = analogRead(infra_BR);
  infras.infraBL = analogRead(infra_BL);
  infras.infraSL = analogRead(infra_SL);

  infras.sonar = ultrasonic.convert( ultrasonic.timing(), Ultrasonic::CM);

  pub_infras.publish(&infras);

  nh.spinOnce();
  delay(25);
}
