#include <Ethernet.h>
#include <PubSubClient.h>  //MQTT Client from Nick O'Leary PubSubClient

byte MAC_ADDRESS[] = { 0x90, 0xA2, 0xDA, 0x0D, 0x51, 0xC8 };

void callback(char* topic, byte* payload, unsigned int length) {
  // handle message arrived
}

char  mqtt_server[] = "192.168.1.12";
char  mqtt_username[] = "xxxxxx";
char  mqtt_password[] = "xxxxxxxx";


EthernetClient ethClient;
PubSubClient mqttclient(mqtt_server, 1883, callback, ethClient);

