#include <WiFi101.h>
#include <PubSubClient.h>  //MQTT Client from Nick O'Leary PubSubClient

char ssid[] = "xxxxxxxxxxx";     //  your network SSID (name)
char pass[] = "xxxxxxxxxxx";    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;                // your network key Index number (needed only for WEP)

char  mqtt_server[] = "xxxxxxxxxx";
char  mqtt_username[] = "xxxxxxxxxx";
char  mqtt_password[] = "xxxxxxxxxxx";

void callback(char* topic, byte* payload, unsigned int length) {
  // handle message arrived
}

int status = WL_IDLE_STATUS;
WiFiClient wificlient;
PubSubClient mqttclient(mqtt_server, 1883, callback, wificlient);

