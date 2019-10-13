/* Esse código é responsável para testar se o servo está respondendo
 *  
 *  
 *  Objetivo: Controlar a posição do servo
 *  
 *  Para fazer o servo girar, , basta abrir o monitor serial e digitar:
 *  1 : soma + 10
 *  2 : soma - 10
 *  
 */
#include <Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

#define SERVO_PIN 6

int pos = 0;    // variable to store the servo position

void setup() {
  Serial.begin(9600);
  myservo.attach(SERVO_PIN);  // attaches the servo on pin 9 to the servo object
  
  delay(500);
  
  int a = myservo.attached ();
  Serial.print(a);
  if( myservo.attached () ) {
    Serial.print("Servo conectado: (aparentemente essa função não funciona ) \n ");
  }
  else{
    Serial.print("O Servo não foi encontrado");
  }
  
  pos = myservo.read();
  Serial.print("Initial pose: ");
  Serial.println(pos);
  

}

void loop() {

}

void serialEvent(){
  char ch = (char)Serial.read();
  
  switch(ch){

    case '1':
        pos = pos + 10;
        break;
    case '2':
        pos = pos - 10;
  }
    Serial.print("Pos: ");
    Serial.print(pos);
    Serial.print(" | ");
    Serial.println(myservo.read());
    
    myservo.write(pos);              
}  
