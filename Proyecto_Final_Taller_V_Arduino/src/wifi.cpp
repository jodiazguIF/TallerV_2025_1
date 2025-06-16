/*
WiFi connection management
*/

#include <WiFiS3.h>
#include "secrets.h"

WiFiUDP udp;
int PORT = 12345;
//Cuando lea desde el udp, voy a ponerlo en una variable tipo char
char Package[255];  //Hasta 255 caracteres 
int dataLen;        //Para saber qué tan grande es el dato

void setupWiFi() {
    Serial.print("Conectándose a ");  //Feedback
  Serial.println(SSID);             //Feedback
  WiFi.begin(SSID,PASS);            //Nos conectamos a la red WiFi
  while(WiFi.status() != WL_CONNECTED){
    //Mientras no está conectado se muestra un puntito cada segundo
    delay(1000);
    Serial.print("."); 
    }
    Serial.println("Conectado a WiFi");
    Serial.println(WiFi.localIP());
    udp.begin(PORT);
    Serial.print("UDP Server started on port: ");
    Serial.println(PORT); 
}