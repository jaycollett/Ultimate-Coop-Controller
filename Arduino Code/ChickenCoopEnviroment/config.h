/************************ Adafruit IO Config *******************************/

// visit io.adafruit.com if you need to create an account,
// or if you need your Adafruit IO key.
#define IO_USERNAME    "XXXXXXXXXXXXXXXX"
#define IO_KEY         "CCCCCCCCCCCCCCCCCCCCCCCC"

/******************************* WIFI **************************************/

// the AdafruitIO_WiFi client will work with the following boards:
//   - HUZZAH ESP8266 Breakout -> https://www.adafruit.com/products/2471
//   - Feather HUZZAH ESP8266 -> https://www.adafruit.com/products/2821
//   - Feather M0 WiFi -> https://www.adafruit.com/products/3010
//   - Feather WICED -> https://www.adafruit.com/products/3056

#define WIFI_SSID       "XXXXXXXXXXXXXX"
#define WIFI_PASS       "XXXXXXXXXXXXXXXXX"

// comment out the following two lines if you are using fona or ethernet
#include <WiFi101.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

WiFi.setPins(8,7,4,2);

char ssid[] = "yournetwork";     //  your network SSID (name)
char pass[] = "yourpassword";    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;                // your network key Index number (needed only for WEP)

int status = WL_IDLE_STATUS;

/************************* Adafruit.io Setup *********************************/

#define AIO_SERVER      "MQTT_SERVER"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "MQTT_USER"
#define AIO_KEY         "MQTT_KEY"

//Set up the wifi client
Adafruit_WINC1500Client client;

Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

// You don't need to change anything below this line!
#define halt(s) { Serial.println(F( s )); while(1);  }
