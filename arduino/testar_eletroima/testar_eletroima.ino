/* Esse código é responsável para testaro eletroima da garra
 *    
 *  Objetivo: Ligar e desligar o eletroima  
 *  
 * Para controlar o eletroima, basta abrir o monitor serial e digitar:
 *  1: Ligar o eletroima
 *  2: Desligar o eletroima
 *  
 */
#define ELETROIMA_PIN 4

void setup()
{
    Serial.begin(9600);

    pinMode(ELETROIMA_PIN, OUTPUT);
}

void loop()
{
}

void serialEvent()
{
    int ch = Serial.read();

    switch (ch)
    {
    case '1': // Liga sensores de linha
        digitalWrite(ELETROIMA_PIN, HIGH);
        Serial.println("turned on");
        break;

    case '2': // Desliga sensores de linha
        digitalWrite(ELETROIMA_PIN, LOW);
        Serial.println("turned off");
        break;
    }
}
