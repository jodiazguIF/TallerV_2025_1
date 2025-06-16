#include <Arduino.h>
#include <WiFiS3.h>
#include <secrets.h>
#include "wifi.cpp"

String PIDstring;
String respuesta;
int LED = 9; //Pin del LED

void setup() {
  Serial.begin(9600); //Iniciamos la comunicación serial
  
  pinMode(LED, OUTPUT); //Configuramos el pin del LED como salida
}

void loop() {
  if (udp.parsePacket()){ //Si hay un paquete listo para ser leído
    //Primero debemos saber qué tan largo es el paquete
    dataLen = udp.available();  //Le asignamos ese valor a la variable
    Serial.println("Largo del paquete: " + dataLen);
    udp.read(Package,255);    //Se lee el paquete y tiene máximo 255 caracteres
    //Puede que haya un montón de basura, entonces vamos a terminar el paquete con un 0
    Package[dataLen]=0 ;
    PIDstring = String(Package);
    PIDstring.trim(); //Elimina espacios en blanco al incio y al final
    Serial.println("PID Recibido = " + PIDstring);  //Respuesta por serial
    respuesta = "Este es el PID para el motor " + PIDstring;
    udp.print(respuesta);
    udp.endPacket();
    if (PIDstring == "1") {
      Serial.println("Encendiendo LED");
      digitalWrite(LED, HIGH); //Encendemos el LED
    }else if (PIDstring == "0") {
      Serial.println("Apagando LED");
      digitalWrite(LED, LOW); //Apagamos el LED
    }
  }
} 
