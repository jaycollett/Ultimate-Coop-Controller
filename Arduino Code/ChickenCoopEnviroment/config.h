#include <WiFi101.h>
#include <PubSubClient.h>  //MQTT Client from Nick O'Leary PubSubClient

char ssid[] = "yournetwork";     //  your network SSID (name)
char pass[] = "yourpassword";    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;                // your network key Index number (needed only for WEP)

char  mqtt_server[] = "yourserverip";
char  mqtt_username[] = "username";
char  mqtt_password[] = "password";

int status = WL_IDLE_STATUS;

WiFiClient wificlient;
PubSubClient mqttclient(wificlient);

