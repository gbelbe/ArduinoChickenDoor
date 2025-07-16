
/*
  ESP8266 Low-Power Motor Control
  This code uses deep sleep to conserve battery.

  ** IMPORTANT HARDWARE MODIFICATION **
  You MUST connect the D0 (GPIO16) pin to the RST pin for deep sleep to work.
*/
#include <Arduino.h>

// Define motor control pins
const int PWMA = D1; // Motor speed
const int DA = D3;   // Motor direction

// Define the light sensor pin
const int LDR_ALIM_PIN = D5;
const int LIGHT_SENSOR_PIN = A0;


// Define how long the ESP8266 will sleep (in minutes)
// Wake up every 15 minutes to check the light. Adjust as needed.
const int SLEEP_INTERVAL_MIN = 1;

// Thresholds for opening/closing the door. Adjust these to your sensor.
const int LIGHT_THRESHOLD_CLOSE = 75;
const int LIGHT_THRESHOLD_OPEN = 100;

// How long to run the motor (in milliseconds). Adjust to your door.
const int MOTOR_RUN_TIME_CLOSE = 6500;
const int MOTOR_RUN_TIME_OPEN = 4300;

// Variable to store the door's state (0 = closed, 1 = open)
// This value is read from RTC memory at boot and saved before sleep.
int isDoorOpen = 0; // Default to closed

void setup() {

  Serial.println("Waking up...");
  Serial.begin(9600); // Keep for debugging, remove for final version
  delay(500); // Allow serial to initialize

  // Set pin modes
  pinMode(LDR_ALIM_PIN, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(DA, OUTPUT);

  //power up the LDR sensor
  digitalWrite(LDR_ALIM_PIN, HIGH);
  // Read the door state from RTC memory
  uint32_t rtcData;
  if (ESP.rtcUserMemoryRead(0, &rtcData, sizeof(rtcData))) {
    isDoorOpen = (int)rtcData;
  }

  // Ensure motor is stopped initially
  digitalWrite(PWMA, LOW);
  digitalWrite(DA, LOW);

  // The main logic is now in setup(), as it runs once on each wake-up.
  runLogic();
}

void runLogic() {
  // Read the light sensor multiple times and average for a stable reading
  int totalLuminosity = 0;
  for (int i = 0; i < 4; i++) {
    totalLuminosity += analogRead(LIGHT_SENSOR_PIN);
    delay(250); // Short delay between readings
  }
  int luminosite = totalLuminosity / 4;

  Serial.print("Luminosity: ");
  Serial.println(luminosite);
  Serial.print("Door is currently: ");
  Serial.println(isDoorOpen == 1 ? "Open" : "Closed");

  // If it's dark and the door is open, close it.
  if (luminosite < LIGHT_THRESHOLD_CLOSE && isDoorOpen == 1) {
    Serial.println("Closing door...");
    closeDoor();
    isDoorOpen = 0; // Update state
  }
  // If it's light and the door is closed, open it.
  else if (luminosite > LIGHT_THRESHOLD_OPEN && isDoorOpen == 0) {
    Serial.println("Opening door...");
    openDoor();
    isDoorOpen = 1; // Update state
  } else {
    Serial.println("No change needed.");
  }

  // Go to sleep to save power
  goToSleep();
}

void openDoor() {
  Serial.println("Motor moving backward (opening)");
  digitalWrite(DA, HIGH); // Set direction
  analogWrite(PWMA, 255); // Set speed
  delay(MOTOR_RUN_TIME_OPEN); // Run motor for specified time
  stopMotor();
}

void closeDoor() {
  Serial.println("Motor moving forward (closing)");
  digitalWrite(DA, LOW); // Set direction
  analogWrite(PWMA, 255); // Set speed
  delay(MOTOR_RUN_TIME_CLOSE); // Run motor for specified time
  stopMotor();
}

void stopMotor() {
  Serial.println("Stopping motor.");
  digitalWrite(PWMA, LOW);
  digitalWrite(DA, LOW);
}

void goToSleep() {
  // Save the current door state to RTC memory before sleeping
  uint32_t rtcData = isDoorOpen;
  ESP.rtcUserMemoryWrite(0, &rtcData, sizeof(rtcData));

  Serial.print("Going to sleep for ");
  Serial.print(SLEEP_INTERVAL_MIN);
  Serial.println(" minutes...");
  // ESP.deepSleep takes microseconds, so convert minutes to microseconds
  ESP.deepSleep(SLEEP_INTERVAL_MIN * 60 * 1000000);
}

// The loop() function is not used because the ESP8266 will deep sleep
// and reset, running setup() again on each wake-up.
void loop() {
  // Intentionally empty
}
