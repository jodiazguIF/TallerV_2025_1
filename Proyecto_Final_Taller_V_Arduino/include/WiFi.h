#ifndef WIFI_H
#define WIFI_H
#include <WiFiS3.h>

extern WiFiUDP udp;           // ← solo declaración
extern int PORT;
extern char Package[255];
extern int dataLen;
extern String PIDstring;
extern String respuesta;
    
// prototipos
void initWiFi();              
void loopWiFi();

#endif