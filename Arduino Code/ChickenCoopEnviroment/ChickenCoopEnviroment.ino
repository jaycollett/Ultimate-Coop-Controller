/********************************************************************************
  Chicken Coop Enviroment Monitor and Controller
  Written by: Jay Collett jay.collett AT gmail.com
  Description: This sketch is for an Adafruit Feather M0 WiFi (WINC1500) and
               is designed to monitor enviromental sensors both inside and
               outside a chicken hen house. All sensor data is sent every
               xx minutes to Adafruit.IO for analyssis. The system leverages
               a featherwing motor control and NEMA 15 motor to control the
               hen house door as well as relays for water heater and fan
               controls. This sketch assumes you are using the custom PCB
               board and pinout.
  Sketch includes unmodified code libraries from the following awesome sources:
      Adafruit.com  https://www.adafruit.com
      sfrwmaker     https://github.com/sfrwmaker/sunMoon
      olikraus      https://github.com/olikraus/u8g2
      jchristensen  https://github.com/JChristensen/Timezone


  This work (excluding external modules/libraries) is licensed under CC BY-SA 4.0

 *********************************************************************************/

// Include all the required libraries
#include "config.h"
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <OneWire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Adafruit_MCP23017.h>
#include <U8g2lib.h>      //https://github.com/olikraus/u8g2
#include <Time.h>
#include <TimeLib.h>
#include <RTClib.h>       //https://github.com/adafruit/RTClib
#include <Timezone.h>     //https://github.com/JChristensen/Timezone
#include <sunMoon.h>      //https://github.com/sfrwmaker/sunMoon


// Define firmware version
#define FIRMWARE_VERSION "v.34"

//#define DEBUG

// Setup our debug printing
#ifdef DEBUG
#define debug(x)     Serial1.print(x)
#define debugln(x)   Serial1.println(x)
#else
#define debug(x)     // define empty, so macro does nothing
#define debugln(x)
#endif

// Define pins for SPI (hardware SPI pins) and whatnot
#define LIGHT_PIN_INSIDE A1
#define LIGHT_PIN_OUTSIDE A3
#define WATER_TEMP_PIN 5
#define WATER_HEATER_CTRL_PIN 12
#define DOOR_CLOSED_PIN A4
#define DOOR_OPENED_PIN A5
#define MANUAL_DOOR_CTRL 11
#define NH3_GAS_PIN A0
#define FAN_CTRL_PIN 6

// Define some other compiler time statics
#define SEALEVELPRESSURE_HPA (1013.25)
#define WATER_HEATER_ON_TEMP 40.0
#define WATER_HEATER_OFF_TEMP 45.0
#define AMBIENT_LUX_DARK 11
#define AMBIENT_LUX_LIGHT 40
#define NH3_BURN_IN_MILLIS 86400000
#define NH3_ALARM_LEVEL 180
#define MIN_OUTSIDE_TEMP 20.0

// Define the pins for our 23017 pin expander (used for debug leds)
#define DOOR_OPEN_LED 0
#define DOOR_MOVING_LED 1
#define DOOR_CLOSED_LED 2
#define FREEZE_WARN_LED 3
#define NH3_WARN_LED 4
#define FAN_ON_LED 5
#define H20_HEAT_LED 6
#define LIGHT_OUT_LED 7
#define DARK_OUT_LED 8
#define BELOW_25_LED 9

// Define our long and lat for use in calc of sunrise/sunset
#define LATITUDE  38.037109
#define LONGITUDE  -84.576166

// Define some images for the display (XBM images; monochrome 1-bit bitmaps)
#define sun_image_width 20
#define sun_image_height 20
static const unsigned char sun_image_bits[] = {
  0x00, 0x06, 0x00, 0x00, 0x06, 0x00, 0x08, 0x06, 0x01, 0x1c, 0x80, 0x01,
  0x18, 0x80, 0x01, 0x80, 0x1f, 0x00, 0xc0, 0x30, 0x00, 0x60, 0x60, 0x00,
  0x20, 0x40, 0x00, 0x27, 0x40, 0x0e, 0x27, 0x40, 0x0e, 0x20, 0x40, 0x00,
  0x60, 0x60, 0x00, 0xc0, 0x30, 0x00, 0x80, 0x1f, 0x00, 0x18, 0x80, 0x01,
  0x1c, 0x80, 0x03, 0x00, 0x06, 0x01, 0x00, 0x06, 0x00, 0x00, 0x06, 0x00
};
#define moon_image_width 20
#define moon_image_height 20
static const unsigned char moon_image_bits[] = {
  0x40, 0x00, 0x00, 0x10, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x0e, 0x00, 0x00, 0x30, 0x00, 0x00, 0x02, 0x00, 0x00, 0x7d, 0x00, 0x00,
  0x01, 0x00, 0x00, 0x52, 0x00, 0x00, 0x56, 0x00, 0x00, 0xa0, 0x00, 0x00,
  0xaf, 0x00, 0x00, 0x40, 0x07, 0x0c, 0x4a, 0x0c, 0x00, 0x94, 0xf3, 0x01,
  0x10, 0x06, 0x03, 0x20, 0xf8, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x1b, 0x00
};
#define fan_image_width 20
#define fan_image_height 20
static const unsigned char fan_image_bits[] = {
  0x00, 0x00, 0x00, 0x00, 0x0f, 0x00, 0xe0, 0x1f, 0x00, 0x70, 0x80, 0x01,
  0x38, 0x80, 0x01, 0x1c, 0x80, 0x01, 0x0c, 0x00, 0x03, 0x04, 0x00, 0x02,
  0x06, 0x00, 0x06, 0x06, 0x00, 0x06, 0x06, 0x00, 0x06, 0x06, 0x00, 0x06,
  0x04, 0x00, 0x02, 0x1c, 0x00, 0x03, 0x1c, 0x80, 0x01, 0x1c, 0xc0, 0x01,
  0x00, 0xe0, 0x00, 0xc0, 0x3f, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x00, 0x00
};
#define heater_image_width 20
#define heater_image_height 20
static const unsigned char heater_image_bits[] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x06, 0x00, 0x20, 0x4f, 0x00, 0xf0, 0xff, 0x00, 0xf0, 0xff, 0x00,
  0xf8, 0xff, 0x01, 0xf8, 0xff, 0x01, 0xf8, 0xf9, 0x01, 0xf8, 0xf0, 0x01,
  0x78, 0xe0, 0x01, 0x70, 0xe0, 0x00, 0x70, 0x70, 0x00, 0xc0, 0x3f, 0x00,
  0x80, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
#define door_open_image_width 20
#define door_open_image_height 20
static const unsigned char door_open_image_bits[] = {
  0x00, 0x00, 0x00, 0xfe, 0x1f, 0x00, 0x06, 0x18, 0x00, 0x02, 0x10, 0x00,
  0x02, 0x10, 0x00, 0x02, 0x10, 0x00, 0x02, 0xc0, 0x00, 0x02, 0xc0, 0x01,
  0x02, 0x80, 0x03, 0x82, 0xff, 0x07, 0x82, 0xff, 0x07, 0x02, 0x80, 0x03,
  0x02, 0xc0, 0x01, 0x02, 0xc0, 0x00, 0x02, 0x10, 0x00, 0x02, 0x10, 0x00,
  0x02, 0x10, 0x00, 0xfe, 0x1f, 0x00, 0xfe, 0x1f, 0x00, 0x00, 0x00, 0x00
};
#define door_closed_image_width 20
#define door_closed_image_height 20
static const unsigned char door_closed_image_bits[] = {
  0x00, 0x00, 0x00, 0x80, 0xff, 0x07, 0x80, 0x01, 0x06, 0x80, 0x00, 0x06,
  0x80, 0x00, 0x06, 0x80, 0x00, 0x06, 0x30, 0x00, 0x06, 0x38, 0x00, 0x06,
  0x1c, 0x00, 0x06, 0xfe, 0x1f, 0x06, 0xfe, 0x1f, 0x06, 0x1c, 0x00, 0x06,
  0x38, 0x00, 0x06, 0x30, 0x00, 0x06, 0x80, 0x00, 0x06, 0x80, 0x00, 0x06,
  0x80, 0x00, 0x06, 0x80, 0xff, 0x07, 0x80, 0xff, 0x07, 0x00, 0x00, 0x00
};

// Setup some consts and other variables needed through the scope of the sketch
const byte waterTempSensorAddr[8] = {0x28, 0xFF, 0x74, 0x3F, 0xA4, 0x16, 0x5, 0xF9};
const long logInterval = 600000;                // interval to update Adafruit IO data (10 mins)
const long tempCheckInterval = 300000;          // internal to check water temp (5 mins)
const long doorCheckInterval = 30000;           // internal to check door status and action (5 mins)
const int doorCloseDelay = 1800;                // (1800 seconds = 30 mins) interval to wait before closing door (allows hens to get in at dark or fully light out)
const long nh3CheckInterval = 600000;           // internal to check door status and action (10 mins)
const long fanCheckInterval = 650000;           // interval to check fan status and action (15 mins)
const long nh3BurnInMillis = 14400000;          // 4 hours before we should be able to get "valid" readings from the sensor
const long doorTransitionTimeout = 45000;       // time in ms to allow door to transition before just shutting down (safety measure)
unsigned long currentMillis;                    // store LOT mcu has been running
unsigned long previousLogMillis = 0;            // store last time we logged data
unsigned long previousTempCheckMillis = 0;      // store last time we checked water temp
unsigned long previousDoorCheckMillis = 0;      // store last time we checked door status
unsigned long previousNH3Millis = 0;            // store last time we logged data
unsigned long previousFanCheckMillis = 0;       // store last time we checked the fan status
unsigned long doorTransitionTimeoutMillis = 0;  // store time we started the door transition

int waterTemp = 99;
int insideTemp = 999;
int outsideTemp = 999;
float currentWaterTemp = 0.0;     // stores the temp for our checks on water temp to control heater
unsigned int inetAttempts = 0;
int updateDisplayDelayCount = 0;  // count display loops
int updateDisplayLoop = 20;      // number of loops to wait till we update OLED display

bool waterHeaterOn = false;       // bool to hold status of water heater
bool doorOpen = false;            // bool to hold status of door
bool nh3BurnInComplete = false;   // hold status of burn-in for nh3 sensor
bool fanOn = false;               // bool to hold status of hen house fan
bool nh3Alarm = false;            // based on nh3 sensor data, is the level too high
bool freezeAlarm = false;         // based on outside temp, set freeze alarm


// Setup our RTC object
RTC_DS3231 rtcObj;

// Setup our timezone object
//US Eastern Time Zone (New York, Detroit)
TimeChangeRule usEDT = {"EDT", Second, Sun, Mar, 2, -240};    //Daylight time = UTC - 4 hours
TimeChangeRule usEST = {"EST", First, Sun, Nov, 2, -300};     //Standard time = UTC - 5 hours
Timezone usEastern(usEDT, usEST);
TimeChangeRule *tcr;        //pointer to the time change rule, use to get TZ abbrev

// Setup sunMoon library object
sunMoon sunMoonObj;

// Setup our 23017 pin expander (uses i2c)
Adafruit_MCP23017 pinExpander;

// Setup a BME280 instance for each of our AdaFruit breakout boards using I2C
Adafruit_BME280 bmeInside;  //addr 0x77
Adafruit_BME280 bmeOutside; //addr 0x76 sdo to gnd

// Water Sensor Address (manual based on each sensor)
OneWire  ds(WATER_TEMP_PIN);  // on pin WATER_TEMP_PIN (a 4.7K resistor is necessary)

// Setting up motorshield objects using i2c
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_StepperMotor *myMotor = AFMS.getStepper(200, 2); // 200 steps per rev on port #2

// Setup our adafruit.io feeds
AdafruitIO_Feed *henhousetemp = io.feed("hen-house-temp");
AdafruitIO_Feed *henhouselightlevel = io.feed("hen-house-light-level");
AdafruitIO_Feed *henhousehumidity = io.feed("hen-house-humidity");
//AdafruitIO_Feed *henhousebarometricpressure = io.feed(" hen-house-barometric-pressure"); we can only have 10 feeds in the adafruit beta, BOOO
AdafruitIO_Feed *henhousenh3levels = io.feed("hen-house-nh3-levels");
AdafruitIO_Feed *chickencoopoutsidetemperature = io.feed("chicken-coop-outside-temperature");
AdafruitIO_Feed *chickencoopoutsidehumidity = io.feed(" chicken-coop-outside-humidity");
AdafruitIO_Feed *chickencoopoutsidelightlevel = io.feed("chicken-coop-outside-light-level");
AdafruitIO_Feed *chickencoopoutsidebarometricpressure = io.feed(" chicken-coop-outside-barometric-pressure");
AdafruitIO_Feed *henhousewatertemperature = io.feed("hen-house-water-temperature");
AdafruitIO_Feed *henhousewaterheaterstat = io.feed("hen-house-water-heater-stat");
AdafruitIO_Feed *henhousedoorstatus = io.feed("hen-house-door-status");
AdafruitIO_Feed *henhousefanstatus = io.feed("hen-house-fan-status");

// Setup our onboard OLED display
U8G2_SH1106_128X64_VCOMH0_F_HW_I2C onBoardDisplay(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);


// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//                                                        SETUP BLOCK
// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
void setup() {

  // starting onboard oled display
  onBoardDisplay.begin();

  // Setup the hw serial for the feather, used for debug...
#ifdef DEBUG
  Serial1.begin(9600);
#endif

  // show boot screen and pause a bit
  // !*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*
  onBoardDisplay.clearBuffer();                         // clear the internal memory
  onBoardDisplay.setFont(u8g2_font_6x10_mr);
  onBoardDisplay.drawStr(0, 10, "Ultimate Coop");       // write something to the internal memory
  onBoardDisplay.drawStr(0, 20, "Controller");          // write something to the internal memory
  onBoardDisplay.drawStr(10, 40, "www.jaycollett.com"); // write something to the internal memory
  onBoardDisplay.drawStr(104, 60, FIRMWARE_VERSION);    // write something to the internal memory
  onBoardDisplay.sendBuffer();                          // transfer internal memory to the display
  delay(5000);
  //!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*

  writeToonBoardDisplay("booting...");

  // Set up our pin modes
  pinMode(LIGHT_PIN_INSIDE, INPUT);
  pinMode(LIGHT_PIN_OUTSIDE, INPUT);
  pinMode(WATER_HEATER_CTRL_PIN, OUTPUT);
  pinMode(DOOR_CLOSED_PIN, INPUT_PULLUP);
  pinMode(DOOR_OPENED_PIN, INPUT_PULLUP);
  pinMode(MANUAL_DOOR_CTRL, INPUT_PULLUP);
  pinMode(NH3_GAS_PIN, INPUT);

  // setup pin expander pin modes
  pinExpander.begin();
  pinExpander.pinMode(DOOR_OPEN_LED, OUTPUT);
  pinExpander.pinMode(DOOR_MOVING_LED, OUTPUT);
  pinExpander.pinMode(DOOR_CLOSED_LED, OUTPUT);
  pinExpander.pinMode(FREEZE_WARN_LED, OUTPUT);
  pinExpander.pinMode(NH3_WARN_LED, OUTPUT);
  pinExpander.pinMode(FAN_ON_LED, OUTPUT);
  pinExpander.pinMode(H20_HEAT_LED, OUTPUT);
  pinExpander.pinMode(LIGHT_OUT_LED, OUTPUT);
  pinExpander.pinMode(DARK_OUT_LED, OUTPUT);
  pinExpander.pinMode(BELOW_25_LED, OUTPUT);


  // Set default output pin states
  digitalWrite(WATER_HEATER_CTRL_PIN, LOW);

  // Setup and init our RTC object
  rtcObj.begin();

  setSyncProvider(syncProvider);   // the function to get the time from the RTC

#ifdef DEBUG
  if (timeStatus() != timeSet)
    debugln("Unable to sync with the RTC or the time hasn't been set on RTC");
  else
    debugln("RTC has set the system time");
#endif

#ifdef DEBUG
  // display the debug UTC and local time
  Serial1.println("Showing UTC from RTC");
  printTime(now(), "UTC");
  Serial1.println("Showing local time from RTC UTC value");
  // display the debug local time and daylight savings abbrev
  printTime(usEastern.toLocal(now(), &tcr), tcr -> abbrev);
#endif

  // validate we can communicate with the sensors...
  bmeInside.begin(0x77);
  bmeOutside.begin(0x76);

  // Connect to Adafruit.IO and wait until connected
  debugln("Connecting to Adafruit IO");
  writeToonBoardDisplay("connecting to inet...");
  io.connect();
  delay(2000);
  while ( (io.status() < AIO_CONNECTED) ) {
    writeToonBoardDisplay("connecting to AIO...");
  }
  
  writeToonBoardDisplay("inet alive...");
  delay(3000);
  
  // set default status for door, fan, and water heater
  henhousewaterheaterstat->save("OFF"); // sending status update to adafruit.io
  delay(1150);
  henhousefanstatus->save("OFF");
  delay(1150);

  // Setup stepper motor control
  AFMS.begin();
  myMotor->setSpeed(90);  // set default RPM for stepper motor

  // Get the status of the hen house door (we don't know if it was open or closed)
  // south pole of magnent is present if low
  if ( (digitalRead(DOOR_CLOSED_PIN) == LOW) && (digitalRead(DOOR_OPENED_PIN) == HIGH)) {
    debugln("Setting inital door state to CLOSED");
    doorOpen = false;
    pinExpander.digitalWrite(DOOR_MOVING_LED, LOW);
    pinExpander.digitalWrite(DOOR_OPEN_LED, LOW);
    pinExpander.digitalWrite(DOOR_CLOSED_LED, HIGH);
    henhousedoorstatus->save("CLOSED"); // sending status update to adafruit.io
  }
  else if ( (digitalRead(DOOR_CLOSED_PIN) == HIGH) && (digitalRead(DOOR_OPENED_PIN) == LOW)) {
    debugln("Setting inital door state to OPEN");
    doorOpen = true;
    pinExpander.digitalWrite(DOOR_MOVING_LED, LOW);
    pinExpander.digitalWrite(DOOR_OPEN_LED, HIGH);
    pinExpander.digitalWrite(DOOR_CLOSED_LED, LOW);
    henhousedoorstatus->save("OPEN"); // sending status update to adafruit.io
  }
  else {
    // well, the door must be stuck inbetween open and close, neither sensor is showing it's position, let's try to close it
    debugln("Couldn't get door status, attempting closeDoor() call..");
    closeDoor();
  }
  writeToonBoardDisplay("finding sensors");
}

// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//                                                           LOOP BLOCK
// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
void loop() {
  // io.run() is required to maintain a connection to adafruit.io
  io.run();

  // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  //  this block of code controls the NH3 (Ammonia) sensor. Sensor needs to be preheated for about 24 hours before reading
  // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  if (!nh3BurnInComplete) {
    if (millis() >= nh3BurnInMillis) {
      // we've been preheating the sensor for the given time, let's set the bool to true
      nh3BurnInComplete = true;
      debugln("Setting NH3 sensor burn-in time to true.");
    }
  } else {
    // preheating was completed, let's see if it's time to grab a reading to send to adafruit.io
    currentMillis = millis();
    if (currentMillis - previousNH3Millis >= nh3CheckInterval) {
      // it's been long enough, let's get the sensor data and send to adafruit.io
      int NH3Reading = readNH3Sensor();
      debug("NH3 Sensor Reading: ");
      debugln(NH3Reading);
      henhousenh3levels->save(NH3Reading); // sending status update to adafruit.io
      delay(1050);

      // determine if the level is too high, if so, set alarm bool
      if (NH3Reading >= NH3_ALARM_LEVEL) {
        pinExpander.digitalWrite(NH3_WARN_LED, HIGH);
        nh3Alarm = true;
      } else {
        pinExpander.digitalWrite(NH3_WARN_LED, LOW);
        nh3Alarm = false;
      }

      previousNH3Millis = currentMillis; // update the time we last updated the log
    }
  }

  // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  //                  this block of code controls the manual open/close function of the henhouse door
  // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  if (digitalRead(MANUAL_DOOR_CTRL) == LOW) {
    debugln("Button is pressed for man door...");
    delay(175); // purposefully don't want to use a more sophisticated millis routine for debounce
    if (digitalRead(MANUAL_DOOR_CTRL) == LOW) {
      // button should have settled down in a "pushed" state, so let's act on the button push
      debug("Door Closed Pin Is: ");
      debugln(digitalRead(DOOR_CLOSED_PIN));
      debug("Door Opened Pin Is: ");
      debugln(digitalRead(DOOR_OPENED_PIN));
      if ( (digitalRead(DOOR_CLOSED_PIN) == LOW) && (digitalRead(DOOR_OPENED_PIN) == HIGH)) {
        debugln("Open Door Based on Manual Button Push");
        openDoor();
      }
      else if ( (digitalRead(DOOR_CLOSED_PIN) == HIGH) && (digitalRead(DOOR_OPENED_PIN) == LOW)) {
        debugln("Close Door Based on Manual Button Push");
        closeDoor();
      }
      else {
        debugln("Unknown state of door, close it");
        closeDoor();
      }
    }
  }


  // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  //          the following code will deal with controlling the water in the hen house based off env sensor data
  // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

  // let's see what the temp is currently for the water in the hen house, if it is close to freezing then let's kick on the heater for a bit to ensure the water doesn't freeze
  // but we only want to check this every so often, defined by tempCheckInterval
  currentMillis = millis();
  if (currentMillis - previousTempCheckMillis >= tempCheckInterval) {
    currentWaterTemp = getWaterTemperature();
    if ((currentWaterTemp <= WATER_HEATER_ON_TEMP) && !waterHeaterOn) {
      // turn on our heater before it's freezes
      digitalWrite(WATER_HEATER_CTRL_PIN, HIGH);
      pinExpander.digitalWrite(H20_HEAT_LED, HIGH);
      waterHeaterOn = true;
      henhousewaterheaterstat->save("ON"); // sending status update to adafruit.io
      delay(1050);
      debug("Current water temp is ");
      debug(currentWaterTemp);
      debugln(" degrees. Turning ON water heater.");
    }
    if ((currentWaterTemp >= WATER_HEATER_OFF_TEMP) && waterHeaterOn) {
      // turn off our water heater, it's warm enough
      digitalWrite(WATER_HEATER_CTRL_PIN, LOW);
      pinExpander.digitalWrite(H20_HEAT_LED, LOW);
      waterHeaterOn = false;
      henhousewaterheaterstat->save("OFF"); // sending status update to adafruit.io
      delay(1050);
      debug("Current water temp is ");
      debug(currentWaterTemp);
      debugln(" degrees. Turning OFF water heater.");
    }
    previousTempCheckMillis = currentMillis; // update the time we last check the water temp
  }

  // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  //                          the following code will deal with controlling the fan in the hen house
  // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  // the fan should stay running when temp is > 90 and cycle on/off when temp is between 80 and 90.
  // we may add logic later to cycle when NH3 is too high but for now, it's temp based
  currentMillis = millis();
  if (currentMillis - previousFanCheckMillis >= fanCheckInterval) {
    if (!fanOn && (Celcius2Fahrenheit(bmeInside.readTemperature()) > 80) && (Celcius2Fahrenheit(bmeInside.readTemperature()) < 90)) {
      digitalWrite(FAN_CTRL_PIN, HIGH);
      fanOn = true;
      pinExpander.digitalWrite(FAN_ON_LED, HIGH);
      henhousefanstatus->save("ON"); // sending status update to adafruit.io
      delay(1050);
    } else if (fanOn && (Celcius2Fahrenheit(bmeInside.readTemperature()) > 80) && (Celcius2Fahrenheit(bmeInside.readTemperature()) < 90)) {
      // cycle fan off
      digitalWrite(FAN_CTRL_PIN, LOW);
      fanOn = false;
      pinExpander.digitalWrite(FAN_ON_LED, LOW);
      henhousefanstatus->save("OFF"); // sending status update to adafruit.io
      delay(1050);
    } else if (fanOn && (Celcius2Fahrenheit(bmeInside.readTemperature()) < 80)) {
      // temp is low enough, turn fan off
      digitalWrite(FAN_CTRL_PIN, LOW);
      fanOn = false;
      pinExpander.digitalWrite(FAN_ON_LED, LOW);
      henhousefanstatus->save("OFF"); // sending status update to adafruit.io
      delay(1050);
    } else if (!fanOn && (Celcius2Fahrenheit(bmeInside.readTemperature()) > 90)) {
      digitalWrite(FAN_CTRL_PIN, HIGH);
      fanOn = true;
      pinExpander.digitalWrite(FAN_ON_LED, HIGH);
      henhousefanstatus->save("ON"); // sending status update to adafruit.io
      delay(1050);
    }
    previousFanCheckMillis = currentMillis; // update the time we last checked the fan status
  }

  // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  //                         the following code will deal with controlling the door to the hen house
  // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

  currentMillis = millis();
  if (currentMillis - previousDoorCheckMillis >= doorCheckInterval) {
    // so first, let's check and set the current status of the door, this will be used later to determine based sunrise/sunset
    // south pole of magnent is present if low
    debugln("Checking door status and sunrise/sunset");
    if ( (digitalRead(DOOR_CLOSED_PIN) == LOW) && (digitalRead(DOOR_OPENED_PIN) == HIGH)) {
      doorOpen = false;
    }
    else if ( (digitalRead(DOOR_CLOSED_PIN) == HIGH) && (digitalRead(DOOR_OPENED_PIN) == LOW)) {
      doorOpen = true;
    }
    //    debug("Door status is currently ");
    //    debugln(doorOpen);
    //    debug("Call to isAfterSunRise() is: ");
    //    debugln(isAfterSunRise());
    //    debug("Call to isAfterSunSet() is: ");
    //    debugln(isAfterSunSet());

    // switching from light sensor logic to RTC & SunRise calculations which don't require internet or have the cons of using
    // a light sensor
    if (doorOpen && isAfterSunSet()) {
      debugln("Door is open and it's after sun set...closing door.");
      pinExpander.digitalWrite(LIGHT_OUT_LED, LOW);
      pinExpander.digitalWrite(DARK_OUT_LED, HIGH);
      closeDoor();
    } else if (!doorOpen && isAfterSunRise() && (Celcius2Fahrenheit(bmeOutside.readTemperature()) > MIN_OUTSIDE_TEMP) && !isAfterSunSet()) {
      debugln("Door is NOT open and it's after sunrise above min outside temp and NOT after sun set.");
      pinExpander.digitalWrite(LIGHT_OUT_LED, HIGH);
      pinExpander.digitalWrite(DARK_OUT_LED, LOW);
      openDoor();
    } else if (!doorOpen && isAfterSunRise() && (Celcius2Fahrenheit(bmeOutside.readTemperature()) <= MIN_OUTSIDE_TEMP && !isAfterSunSet())) {
      debugln("Door is NOT open and it's after sunrise, NOT after sun set and TOO COLD to open door..");
      pinExpander.digitalWrite(BELOW_25_LED, HIGH); // we cant open the door because it's too cold outside, otherwise it'd be opening...set status light
    }

    previousDoorCheckMillis = currentMillis; // update the time we last checked the door status
  }

  // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  //                      the following code will evaulate the temp and set a freeze warning bool and led
  // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  if (Celcius2Fahrenheit(bmeOutside.readTemperature()) < 34) {
    debugln("Set freeze alarm flag");
    pinExpander.digitalWrite(FREEZE_WARN_LED, HIGH);
    freezeAlarm = true;
  } else {
    pinExpander.digitalWrite(FREEZE_WARN_LED, LOW);
    freezeAlarm = false;
    debugln("No freeze alarm, disable flag");
  }

  // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  //                        the following code will update our onboard oled display with the defined data
  // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  if (updateDisplayDelayCount >= updateDisplayLoop) {
    debugln("Updating oled screen...");
    onBoardDisplayUpdate();
    // io.run() is required to maintain a connection to adafruit.io
    io.run();
    updateDisplayDelayCount = 0;
  } else {
    updateDisplayDelayCount++;
  }

  // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  // this block of code controls the actual updates of env data to adafruit.io, once every xx mins as deinfed by logInterval
  // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  currentMillis = millis();

  if (currentMillis - previousLogMillis >= logInterval) {

    // first make sure we are connected to the network and AdaFruit.io
    // if not, try for 120 seconds and then reboot the feather
    int connectAttempts = 0;
    while ( (io.status() < AIO_CONNECTED) ) {
      if (connectAttempts >= 12) {
        connectAttempts = 0;
        WiFi.disconnect();
        WiFi.end();
        NVIC_SystemReset();
        delay(2000);
        WiFi.begin();
      }
      debugln("Connection seems to have been disconnected, attempting reconnect.");
      debug("Error is: ");
      debugln(io.status());
      WiFi.end();
      io.connect();
      delay(10000);
      connectAttempts++;
    }
    // io.run() is required to maintain a connection to adafruit.io
    io.run();
    updateBMEInsideData(bmeInside); // Update adafruit.io with current hen house enviromental readings
    updateInsideLightLevel();
    // io.run() is required to maintain a connection to adafruit.io
    io.run();
    updateBMEOutsideData(bmeOutside); // Update adafruit.io with the current ambient sensor readings
    updateOutsideLightLevel();

    // update water temp to adafruit.io
    delay(1050);
    henhousewatertemperature->save(getWaterTemperature());
    delay(1050);
    // print to the serial port the data we are attempting to send to Adafruit.IO for debugging
#ifdef DEBUG
    printBMEData(bmeInside); // Use a simple function to print out the data (testing)
    printLightLevel(LIGHT_PIN_INSIDE);
    printBMEData(bmeOutside); // Use a simple function to print out the data (testing)
    printLightLevel(LIGHT_PIN_OUTSIDE);
#endif

    previousLogMillis = currentMillis; // update the time we last updated the log
  }
}

// &$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$
// &$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$
//                                                      methods for main loop use are below
// &$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$
// &$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$&$

// #############################################################################
//  readNH3Sensor
//  This method will read the NH3 sensor analog value and return a float value
//  which is based on the amount of detectable NH3 gas.
// #############################################################################
float readNH3Sensor() {
  float sensorReadings = 0.0;
  sensorReadings = analogRead(NH3_GAS_PIN);
  delay(600);
  sensorReadings += analogRead(NH3_GAS_PIN);
  delay(600);
  sensorReadings += analogRead(NH3_GAS_PIN);

  return (sensorReadings / 3.0);
}

// #############################################################################
// closeDoor
// This method is responsible for controlling the stepper motor to close the
// hen house door as well as updating the LED status indicators
// #############################################################################
void closeDoor() {
  doorTransitionTimeoutMillis = millis(); //curent time to compare against our door open/close timeout

  // step "forward" to close the door by 10 steps then check to see if we are closed, if not loop
  debugln("Close door method called...");
  pinExpander.digitalWrite(DOOR_MOVING_LED, HIGH);
  debugln("turned on door moving led...");
  debugln("closing door...");
  bool doorTransitionTimedOut = false;
  
  while ((digitalRead(DOOR_CLOSED_PIN) == HIGH) && ( !doorTransitionTimedOut )) {
    myMotor->step(10, FORWARD, DOUBLE);
    // check to see if the door has been moving for too long, based on the doorTransitionTimeout value
    if ( (doorTransitionTimeoutMillis + doorTransitionTimeout) < millis( )){
      doorTransitionTimedOut = true;
    }
  }
  if(!doorTransitionTimedOut){
    doorOpen = false;
    myMotor->release(); //this remove holding torque by cutting power to the coils...keeps the motor from getting wicked hot
    pinExpander.digitalWrite(DOOR_MOVING_LED, LOW);
    pinExpander.digitalWrite(DOOR_OPEN_LED, LOW);
    pinExpander.digitalWrite(DOOR_CLOSED_LED, HIGH);
    henhousedoorstatus->save("CLOSED"); // sending status update to adafruit.io
    delay(1050);
  }else{
    doorOpen = true;
    myMotor->release(); //this remove holding torque by cutting power to the coils...keeps the motor from getting wicked hot
    pinExpander.digitalWrite(DOOR_MOVING_LED, HIGH);
    pinExpander.digitalWrite(DOOR_OPEN_LED, LOW);
    pinExpander.digitalWrite(DOOR_CLOSED_LED, LOW);
    henhousedoorstatus->save("ERROR CLOSING"); // sending status update to adafruit.io
    delay(1050);
  }
}

// #############################################################################
// openDoor
// This method is responsible for controlling the stepper motor to open the
// hen house door as well as updating the LED status indicators
// #############################################################################
void openDoor() {
  // step "backwards" to open the door by 10 steps then check to see if the door is opened, if not loop

  doorTransitionTimeoutMillis = millis(); //curent time to compare against our door open/close timeout
  pinExpander.digitalWrite(DOOR_MOVING_LED, HIGH);
  bool doorTransitionTimedOut = false;

  while ( (digitalRead(DOOR_OPENED_PIN) == HIGH) && ( !doorTransitionTimedOut )) {
    myMotor->step(10, BACKWARD, DOUBLE);
    if ( (doorTransitionTimeoutMillis + doorTransitionTimeout) < millis() ){
      doorTransitionTimedOut = true;
    }
  }
  if(!doorTransitionTimedOut){
    doorOpen = true;
    myMotor->release(); //this remove holding torque by cutting power to the coils...keeps the motor from getting wicked hot
    pinExpander.digitalWrite(BELOW_25_LED, LOW);
    pinExpander.digitalWrite(DOOR_CLOSED_LED, LOW);
    pinExpander.digitalWrite(DOOR_MOVING_LED, LOW);
    pinExpander.digitalWrite(DOOR_OPEN_LED, HIGH);
    henhousedoorstatus->save("OPEN"); // sending status update to adafruit.io
    delay(1050);
  }else{
    doorOpen = false;
    myMotor->release(); //this remove holding torque by cutting power to the coils...keeps the motor from getting wicked hot
    pinExpander.digitalWrite(BELOW_25_LED, LOW);
    pinExpander.digitalWrite(DOOR_CLOSED_LED, LOW);
    pinExpander.digitalWrite(DOOR_MOVING_LED, HIGH);
    pinExpander.digitalWrite(DOOR_OPEN_LED, LOW);
    henhousedoorstatus->save("ERROR OPENING"); // sending status update to adafruit.io
    delay(1050);
  }
}

// #############################################################################
// updateInsideLightLevel
// This method is responsible for sending the light level detected by the
// inside sensor to the cloud IOT system.
// #############################################################################
void updateInsideLightLevel() {
  henhouselightlevel->save(getLightLevelReading(LIGHT_PIN_INSIDE));
  delay(1150);
}

// #############################################################################
// updateOutsideLightLevel
// This method is responsible for sending the light level detected by the
// outside sensor to the cloud IOT system.
// #############################################################################
void updateOutsideLightLevel() {
  chickencoopoutsidelightlevel->save(getLightLevelReading(LIGHT_PIN_OUTSIDE));
  delay(1150);
}

// #############################################################################
// printLightLevel
// This method is is used for debug purposes and prints to Serial1 the analog
// value from the light sensor pin supplied.
// #############################################################################
void printLightLevel(int lightSensorPin) {
  Serial1.print("Light level reading is ");
  Serial1.println(getLightLevelReading(lightSensorPin));
}

// #############################################################################
// updateBMEInsideData
// This method is used to submit the temp and humdity data to the cloud IOT
// system for the interior sensor.
// #############################################################################
void updateBMEInsideData(Adafruit_BME280 sensorName) {
  henhousetemp->save(Celcius2Fahrenheit(sensorName.readTemperature()));
  delay(1150);
  henhousehumidity->save(sensorName.readHumidity());
  delay(1150);
}

// #############################################################################
// updateBMEOutsideData
// This method is used to submit the temp and humdity data to the cloud IOT
// system for the exterior sensor.
// #############################################################################
void updateBMEOutsideData(Adafruit_BME280 sensorName) {
  chickencoopoutsidetemperature->save(Celcius2Fahrenheit(sensorName.readTemperature()));
  delay(1150);
  chickencoopoutsidehumidity->save(sensorName.readHumidity());
  delay(1150);
  chickencoopoutsidebarometricpressure->save(sensorName.readPressure() / 3386.39F); // we want inches-Hg there are 3,386.39 Pascals to 1 inch Hg
  delay(1150);
}

// #############################################################################
// printBMEData
// This method is used for debugging purposes and prints the humdity and temp
// sensor data to the serial1 given a BME sensor object.
// #############################################################################
void printBMEData(Adafruit_BME280 sensorName)
{
  Serial1.print("Temperature = ");
  Serial1.print(Celcius2Fahrenheit(sensorName.readTemperature()));
  Serial1.println(" *F");

  Serial1.print("Pressure = ");

  //Serial.print(sensorName.readPressure() / 100.0F);
  Serial1.print(sensorName.readPressure() / 3386.39F);
  Serial1.println(" inches-Hg");

  Serial1.print("Approx. Altitude = ");
  Serial1.print(sensorName.readAltitude(SEALEVELPRESSURE_HPA));
  Serial1.println(" m");

  Serial1.print("Humidity = ");
  Serial1.print(sensorName.readHumidity());
  Serial1.println(" %");

  Serial1.println();
}

float getWaterTempSensorValue() {
  byte present = 0;
  byte data[12];
  float celsius, fahrenheit;
  byte i;
  byte type_s;

  ds.reset();
  ds.select(waterTempSensorAddr);
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end

  delay(775);     // maybe 750ms is enough, maybe not

  present = ds.reset();
  ds.select(waterTempSensorAddr);
  ds.write(0xBE);         // Read Scratchpad

  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
  }
  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }
  celsius = (float)raw / 16.0;
  fahrenheit = celsius * 1.8 + 32.0;
  return fahrenheit;
}

float getWaterTemperature()
{
  float tempSum = 0.0;
  float avgTemp = 0.0;

  // get the temp 3 times and average it out, wait a bit between to allow sensor to respond
  tempSum += getWaterTempSensorValue();
  delay(50);
  tempSum += getWaterTempSensorValue();

  avgTemp = tempSum / 2.0;

  return avgTemp;
}

float Celcius2Fahrenheit(float celsius)
{
  return (celsius * 9.0 / 5.0) + 32.0;
  //  if (celsius < 0) return (celsius * 18 - 50)/100 + 32; else return (celsius * 18 + 50)/100 + 32;
}

int getLightLevelReading(int analogLightSensorPin) {
  int readingSum = 0;
  for (int readings = 0; readings <= 2; readings++) {
    readingSum = readingSum  + analogRead(analogLightSensorPin);
    delay(50);
  }
  return readingSum / 3; // averaging the analog reading from the light sensor
}

void onBoardDisplayUpdate() {

  onBoardDisplay.clearBuffer();

  // get internal and external temps and draw them
  drawIntTemp(round(Celcius2Fahrenheit(bmeInside.readTemperature())));
  drawExtTemp(round(Celcius2Fahrenheit(bmeOutside.readTemperature())));

  // get water temp and draw it
  drawH20Temp(round(getWaterTemperature()));

  // draw sun/moon based on brightness level of outside
  drawSunMoonStatus();

  // if fan is on, draw the fan graphic
  drawFanStatus();

  // if water heater is on, draw the heater/flame icon
  drawWaterHeaterStatus();

  // draw the door status graphic
  drawDoorStatus();

  // draw nh3 alarm if level is too high
  //drawnh3Alarm();

  // draw feeze alam if temp outside is low enough
  drawFreezeAlarm();

  onBoardDisplay.sendBuffer();
}

void drawFreezeAlarm() {
  if (freezeAlarm) {
    // DISPLAY nh3 alarm text
    onBoardDisplay.setFont(u8g2_font_6x10_mr);
    onBoardDisplay.setCursor(92, 35); // right aligned, center row
    onBoardDisplay.print("FREEZE");
  }
}

void drawnh3Alarm() {
  if (nh3Alarm) {
    // DISPLAY nh3 alarm text
    onBoardDisplay.setFont(u8g2_font_6x10_mr);
    onBoardDisplay.setCursor(0, 35);  // left aligned, center row
    onBoardDisplay.print("NH3 ALARM");
  }
}

void drawDoorStatus() {
  if (doorOpen) {
    onBoardDisplay.drawXBM(0, 0, door_open_image_width, door_open_image_height, door_open_image_bits);
  } else {
    onBoardDisplay.drawXBM(0, 0, door_closed_image_width, door_closed_image_height, door_closed_image_bits);
  }
}

void drawWaterHeaterStatus() {
  if (waterHeaterOn) {
    onBoardDisplay.drawXBM(68, 0, heater_image_width, heater_image_height, heater_image_bits);
  }
}

void drawFanStatus() {
  if (fanOn) {
    onBoardDisplay.drawXBM(36, 0, fan_image_width, fan_image_height, fan_image_bits);
  }
}

void drawSunMoonStatus() {
  //  debug("Checking outside light level: ");
  //  debug(getLightLevelReading(LIGHT_PIN_OUTSIDE));
  //  debug(" and ambient value for dark is: ");
  //  debugln(AMBIENT_LUX_DARK);
  if (getLightLevelReading(LIGHT_PIN_OUTSIDE) >= AMBIENT_LUX_LIGHT) {
    onBoardDisplay.drawXBM(100, 0, sun_image_width, sun_image_height, sun_image_bits);
  } else {
    onBoardDisplay.drawXBM(100, 0, moon_image_width, moon_image_height, moon_image_bits);
  }
}

void drawH20Temp(int temp) {
  // BUILD FUNCTION TO DRAW h20 TEMP ON SCREEN
  onBoardDisplay.setFont(u8g2_font_6x10_mr);
  onBoardDisplay.setCursor(48, 64);
  onBoardDisplay.print("W: ");
  onBoardDisplay.setCursor(66, 64);
  onBoardDisplay.print(temp);
}

void drawIntTemp(int temp) {
  onBoardDisplay.setFont(u8g2_font_6x10_mr);
  onBoardDisplay.setCursor(0, 64);
  onBoardDisplay.print("I: ");
  onBoardDisplay.setCursor(26, 64);
  onBoardDisplay.print(temp); //drop decimals
}

void drawExtTemp(int temp) {
  int tempLen;
  onBoardDisplay.setFont(u8g2_font_6x10_mr);

  if ((temp < -9) || (temp > 99)) {
    tempLen = 18;
  } else if (temp < 0) {
    tempLen = 12;
  } else if (temp < 10) {
    tempLen = 6;
  } else {
    tempLen = 12;
  }
  onBoardDisplay.setCursor((128 - (tempLen + 26)), 64);
  onBoardDisplay.print("O: ");
  onBoardDisplay.setCursor((128 - tempLen), 64);
  onBoardDisplay.print(temp);
}

void writeToonBoardDisplay(String textToWrite) {
  int strLen = textToWrite.length() + 1; // get length plus 1 for null terminator
  char charToWrite[strLen];
  textToWrite.toCharArray(charToWrite, strLen);

  onBoardDisplay.clearBuffer();          // clear the internal memory
  onBoardDisplay.setFont(u8g2_font_6x10_mr);
  onBoardDisplay.drawStr(10, 40, charToWrite); // write something to the internal memory
  onBoardDisplay.sendBuffer();          // transfer internal memory to the display
  //!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*
}

bool isAfterSunSet() {
  debugln("Call to isAfterSunSet()");

  // set variable to current local time (follows DST rules we defined earlier)
  time_t currentLocalTime  = usEastern.toLocal(now(), &tcr);

  // first, let's determine if we are in DST or not based on our DST rules we defined earlier in code
  if (usEastern.locIsDST(currentLocalTime)) {
    // we are in summer time -4 hours utc
    debugln("Check shows we are in -4 hours, summer time.");
    sunMoonObj.init(-240, LATITUDE, LONGITUDE);
  } else {
    // we are in winter time -5 hours utc
    debugln("Check shows we are in -5 hours, winter time.");
    sunMoonObj.init(-300, LATITUDE, LONGITUDE);
  }

  //  debug("Today's sunset time is: ");
  //  debugln(sunMoonObj.sunSet(currentLocalTime));
  //  debug("We should close the door at (based on value of doorCloseDelay): ");
  //  printDate(sunMoonObj.sunSet(currentLocalTime) + doorOpenCloseDelay);

  // decide if we are past our sunset threshold to act on the door
  if ((sunMoonObj.sunSet(currentLocalTime) + doorCloseDelay) <= currentLocalTime) {
    debugln("isAfterSunSet() call returned TRUE");
    return true;
  } else {
    debugln("isAfterSunSet() call returned FALSE");
    return false;
  }
}

bool isAfterSunRise() {
  debugln("Call to isAfterSunRise()");

  // set variable to current local time (follows DST rules we defined earlier)
  time_t currentLocalTime  = usEastern.toLocal(now(), &tcr);

  // first, let's determine if we are in DST or not based on our DST rules we defined earlier in code
  if ( usEastern.locIsDST(currentLocalTime) ) {
    // we are in summer time -4 hours utc
    debugln("Check shows we are in -4 hours, summer time.");
    sunMoonObj.init(-240, LATITUDE, LONGITUDE);
  } else {
    // we are in winter time -5 hours utc
    debugln("Check shows we are in -5 hours, winter time.");
    sunMoonObj.init(-300, LATITUDE, LONGITUDE);
  }

  //  debug("Today's sunrise time is: ");
  //  debugln(sunMoonObj.sunRise(currentLocalTime));
  //  debug("We should open the door at (based on value of doorOpenCloseDelay): ");
  //  printDate(sunMoonObj.sunRise(currentLocalTime) + doorOpenCloseDelay);

  // decide if we are past our sunrise threshold to act on the door ( no delay here, open when the sun's up!)
  if ( (sunMoonObj.sunRise(currentLocalTime) ) <= currentLocalTime ) {
    debugln("isAfterSunRise() call returned TRUE");
    return true;
  } else {
    debugln("isAfterSunRise() call returned FALSE");
    return false;
  }
}

// Function to provide RTC sync
time_t syncProvider()     //this does the same thing as RTC_DS1307::get() but the adafruit library has no get() method
{
  return rtcObj.now().unixtime();
}

//Function to print time with time zone
void printTime(time_t t, char *tz)
{
  sPrintI00(hour(t));
  sPrintDigits(minute(t));
  sPrintDigits(second(t));
  Serial1.print(' ');
  Serial1.print(dayShortStr(weekday(t)));
  Serial1.print(' ');
  sPrintI00(day(t));
  Serial1.print(' ');
  Serial1.print(monthShortStr(month(t)));
  Serial1.print(' ');
  Serial1.print(year(t));
  Serial1.print(' ');
  Serial1.print(tz);
  Serial1.println();
}

//Print an integer in "00" format (with leading zero).
//Input value assumed to be between 0 and 99.
void sPrintI00(int val)
{
  if (val < 10) Serial1.print('0');
  Serial1.print(val, DEC);
  return;
}

//Print an integer in ":00" format (with leading zero).
//Input value assumed to be between 0 and 99.
void sPrintDigits(int val)
{
  Serial1.print(':');
  if (val < 10) Serial1.print('0');
  Serial1.print(val, DEC);
}

void printDate(time_t date) {
  char buff[20];
  sprintf(buff, "%2d-%02d-%4d %02d:%02d:%02d",
          day(date), month(date), year(date), hour(date), minute(date), second(date));
  Serial1.print(buff);
}

