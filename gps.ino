/**
 *  ___ ___  _  _   _     ___  __  ___    ___ ___  ___
 * | __/ _ \| \| | /_\   ( _ )/  \( _ )  / __| _ \/ __|
 * | _| (_) | .` |/ _ \  / _ \ () / _ \ | (_ |  _/\__ \
 * |_| \___/|_|\_/_/ \_\ \___/\__/\___/  \___|_|  |___/
 *
 * This example is meant to work with the Adafruit
 * FONA 808 or 3G Shield or Breakout
 *
 * Copyright: 2015 Adafruit
 * Author: Todd Treece
 * Licence: MIT
 *
 */
#include "Adafruit_FONA.h"

// standard pins for the shield, adjust as necessary
// gris rx fona
#define FONA_RX 3
// rose tx fona
#define FONA_TX 4
// bleu
#define FONA_RST 5

#define GPS_URL "http://adellia.yoda-bzh.net/gps.php"
//#define GSM_APM "free"
#define GSM_APN "slsfr"

#define PIN_LED 13

// We default to using software serial. If you want to use hardware serial
// (because softserial isnt supported) comment out the following three lines 
// and uncomment the HardwareSerial line
#include <SoftwareSerial.h>
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;

// Hardware serial is also possible!
//  HardwareSerial *fonaSerial = &Serial1;

Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

// Have a FONA 3G? use this object type instead
//Adafruit_FONA_3G fona = Adafruit_FONA_3G(FONA_RST);

void checkConnection() {

  uint8_t n = 0;
  Serial.println("Going to check network status");
  do {
    n = fona.getNetworkStatus();
    if (1 == n) {
      Serial.println("We seems to be registered, good");
      break;
    }
    if (n == 0) Serial.println(F("Not registered"));
    if (n == 2) Serial.println(F("Not registered (searching)"));
    if (n == 3) Serial.println(F("Denied"));
    if (n == 4) Serial.println(F("Unknown"));
    if (n == 5) Serial.println(F("Registered roaming"));
    delay(2000);
  } while (n != 1);
  
  Serial.print(F("good ! Network status "));
  Serial.print(n);
  Serial.print(F(": "));
  if (n == 0) Serial.println(F("Not registered"));
  if (n == 1) Serial.println(F("Registered (home)"));
  if (n == 2) Serial.println(F("Not registered (searching)"));
  if (n == 3) Serial.println(F("Denied"));
  if (n == 4) Serial.println(F("Unknown"));
  if (n == 5) Serial.println(F("Registered roaming"));
}

void setup() {
  //pinMode(PIN_LED, OUTPUT);
  //digitalWrite(PIN_LED, HIGH);
  //delay(100);
  //digitalWrite(PIN_LED, LOW);
  //delay(100);

  while (! Serial);

  Serial.begin(115200);
  Serial.println(F("Adafruit FONA 808"));
  Serial.println(F("Initializing FONA... (May take a few seconds)"));

  //digitalWrite(PIN_LED, HIGH);
  //delay(100);
  //digitalWrite(PIN_LED, LOW);
  //delay(100);
  //digitalWrite(PIN_LED, HIGH);
  //delay(100);
  //digitalWrite(PIN_LED, LOW);
  //delay(100);

  fonaSerial->begin(4800);
  if (! fona.begin(*fonaSerial)) {
    Serial.println(F("Couldn't find FONA"));
    while(1);
  }
  Serial.println(F("Setting APN"));
  fona.setGPRSNetworkSettings(F(GSM_APN));
  
  Serial.println(F("FONA is OK"));

  checkConnection();

  //if(!fona.enableGPS(true)) {
  //  Serial.println("Unable to enable GPS");
  //}
  do {
    Serial.println(F("Enabling GPS..."));
    delay(2000);
  } while(!fona.enableGPS(true));
  
  // Try to enable GPRS

  do {
    Serial.println("Trying to enable GPRS");
    delay(2000);
  } while(!fona.enableGPRS(true));
  
  //if (!fona.enableGPRS(true)) {
  //  Serial.println(F("Failed to turn on gprs"));
  //  while(1);
  //}
}

void loop() {
  delay(20000); // 10 secondes

  float latitude, longitude, speed_kph, heading, speed_mph, altitude;
  char s_latitude[15], s_longitude[15], s_speek_kph[10], s_heading[10];
  
  //char url[] = "http://adellia.yoda-bzh.net/gps.php";
  char data[100];
  uint16_t statuscode;
  int16_t length;

  Serial.println(F("Getting GPS"));
  // if you ask for an altitude reading, getGPS will return false if there isn't a 3D fix
  boolean gps_success = fona.getGPS(&latitude, &longitude, &speed_kph, &heading, &altitude);

  if (!gps_success) {
    Serial.println(F("Unable to get GPS pos"));
    //return;
  }

  // data += longitude;
  // data += ";";
  // data += latitude;
  // data += ";";
  // data += speed_kph;
  // data += ";";
  // data += heading;
  //sprintf(data, "%.9f;%.9f;%.9f;%.9f", longitude, latitude, speed_kph, heading);
  dtostrf(longitude, 14, 5, s_longitude );
  dtostrf(latitude,  14, 5, s_latitude  );
  dtostrf(speed_kph, 9,  5, s_speek_kph );
  dtostrf(heading,   9,  5, s_heading   );
  sprintf(
      data, 
      "lo=%s&la=%s&s=%s&h=%s", 
      s_longitude,
      s_latitude,
      s_speek_kph,
      s_heading
  );

  Serial.println(data);

  checkConnection();
  Serial.println(F("Posting data"));
  if(fona.HTTP_POST_start(GPS_URL, F("application/x-www-form-urlencoded"), (uint8_t *) data, strlen(data), &statuscode, (uint16_t *) &length)) {
    Serial.println("Failed to post http data!");
    return;
  }
  fona.HTTP_POST_end();
  Serial.println(F("done."));
}
