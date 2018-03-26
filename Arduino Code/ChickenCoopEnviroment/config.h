#include <Ethernet2.h>
#include <PubSubClient.h>  //MQTT Client from Nick O'Leary PubSubClient

byte mac[] = { 0xDE, 0xFD, 0xBE, 0xEF, 0xFE, 0xED };  

void callback(char* topic, byte* payload, unsigned int length) {
  // handle message arrived
}

char  mqtt_server[] = "192.168.0.5";
char  mqtt_username[] = "chickenCoop";
char  mqtt_password[] = "chickenClient01!";
char  mqtt_clientid[] = "chickenCoop";

#define WIZ_CS 10

EthernetClient ethClient;
// in case dhcp fails
IPAddress dnsServer(192, 168, 0, 20);
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress ip(192, 168, 0, 244);


PubSubClient mqttclient(mqtt_server, 1883, callback, ethClient);

