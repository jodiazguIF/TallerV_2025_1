/*
WiFi connection management
*/

#include <WiFiS3.h>
#include "secrets.h"
#include "WiFi.h" 

WiFiUDP udp;                  
int PORT = 12345;
char Package[255];
int dataLen = 0;
String PIDstring;
String respuesta;

void initWiFi() {
    Serial.print("Conectándose a ");  //Feedback
    Serial.println(SSID);             //Feedback
    WiFi.begin(SSID,PASS);            //Nos conectamos a la red WiFi
    while(WiFi.status() != WL_CONNECTED){
    //Mientras no está conectado se muestra un puntito cada segundo
    delay(100);
    Serial.print("."); 
  }
  Serial.println("Conectado a WiFi");
  Serial.println(WiFi.localIP());
  udp.begin(PORT);
  Serial.print("UDP Server started on port: ");
  Serial.println(PORT); 
}

void loopWiFi(){
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
      // Cosas
    }else if (PIDstring == "0") {
      // Cosas
    }
  }
}