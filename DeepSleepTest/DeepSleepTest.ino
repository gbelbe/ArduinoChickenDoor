/*
  ESP8266 Deep Sleep Test
  This sketch tests the basic deep sleep functionality of the ESP8266.

  ** IMPORTANT HARDWARE MODIFICATION **
  You MUST connect the D0 (GPIO16) pin to the RST pin for deep sleep to work.
*/

#include <Arduino.h>
#include <ESP.h>

// Define how long the ESP8266 will sleep (in microseconds)
// 5 seconds for testing purposes
const int SLEEP_DURATION_US = 5 * 1000000;

void setup() {
   delay(1000);
  Serial.begin(9600);
  // Give some time for the serial monitor to connect
  delay(1000);
  Serial.println("--- Deep Sleep Test ---");
  Serial.println("Waking up...");
  Serial.print("Going to sleep for ");
  Serial.print(SLEEP_DURATION_US / 1000000);
  Serial.println(" seconds...");

  // Go to deep sleep
  ESP.deepSleep(SLEEP_DURATION_US);
}

void loop() {
  // This loop will not be reached as the ESP8266 goes into deep sleep from setup()
}
