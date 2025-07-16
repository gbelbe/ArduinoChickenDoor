/*
  ESP8266 Low-Power Motor Control for L298N Driver

  L298N WIRING:
  - L298N VCC/12V: Connect to your battery's positive terminal.
  - L298N GND:      Connect to your battery's negative terminal AND a GND pin on the ESP8266.
  - L298N OUT1/OUT2: Connect to the two terminals of your DC motor.

  - ESP8266 D1: Connect to L298N ENA (Enable A for speed control).
  - ESP8266 D3: Connect to L298N IN1 (Direction control 1).
  - ESP8266 D4: Connect to L298N IN2 (Direction control 2).

  ** IMPORTANT HARDWARE MODIFICATION **
  You MUST connect the D0 (GPIO16) pin to the RST pin for deep sleep to work.
*/
#include <Arduino.h>
#include <ESP8266WiFi.h>

// Uncomment the next line to enable Serial debugging
#define DEBUG

#ifdef DEBUG
  #define SERIAL_PRINT(x) Serial.print(x)
  #define SERIAL_PRINTLN(x) Serial.println(x)
  const int SLEEP_INTERVAL_MIN = 1; // Sleep for 1 minute in debug mode
#else
  #define SERIAL_PRINT(x)
  #define SERIAL_PRINTLN(x)
  const int SLEEP_INTERVAL_MIN = 15; // Sleep for 15 minutes in production
#endif

// Define motor control pins for L298N
const int MOTOR_ENA = D1; // Speed control (PWM)
const int MOTOR_IN1 = D3; // Direction control 1
const int MOTOR_IN2 = D4; // Direction control 2

// Define the light sensor pin
const int LDR_ALIM_PIN = D5;
const int LIGHT_SENSOR_PIN = A0;

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
  #ifdef DEBUG
    Serial.begin(9600);
    delay(500); // Allow serial to initialize
  #endif

  // Check if the reset button was pressed. If so, clear the RTC memory.
  rst_info *resetInfo = ESP.getResetInfoPtr();
  if (resetInfo->reason == REASON_EXT_SYS_RST) {
    SERIAL_PRINTLN("RST button pressed. Clearing RTC memory...");
    uint32_t resetValue = 0; // Reset to 0 (Closed)
    ESP.rtcUserMemoryWrite(0, &resetValue, sizeof(resetValue));
    pinMode(LED_BUILTIN, OUTPUT);
    for(int i=0; i<5; i++){
      digitalWrite(LED_BUILTIN, LOW);
      delay(50);
      digitalWrite(LED_BUILTIN, HIGH);
      delay(50);
    }
    delay(2000); // Pause to show confirmation
  }

  SERIAL_PRINTLN("Waking up to run main logic...");

  // Set pin modes
  pinMode(LDR_ALIM_PIN, OUTPUT);
  pinMode(MOTOR_ENA, OUTPUT);
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);

  //power up the LDR sensor
  digitalWrite(LDR_ALIM_PIN, HIGH);
  // Read the door state from RTC memory
  uint32_t rtcData;
  if (ESP.rtcUserMemoryRead(0, &rtcData, sizeof(rtcData))) {
    isDoorOpen = (int)rtcData;
  }

  // Ensure motor is stopped initially
  stopMotor();

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

  SERIAL_PRINT("Luminosity: ");
  SERIAL_PRINTLN(luminosite);
  SERIAL_PRINT("Door is currently: ");
  SERIAL_PRINTLN(isDoorOpen == 1 ? "Open" : "Closed");

  // If it's dark and the door is open, close it.
  if (luminosite < LIGHT_THRESHOLD_CLOSE && isDoorOpen == 1) {
    SERIAL_PRINTLN("Closing door...");
    closeDoor();
    isDoorOpen = 0; // Update state
  }
  // If it's light and the door is closed, open it.
  else if (luminosite > LIGHT_THRESHOLD_OPEN && isDoorOpen == 0) {
    SERIAL_PRINTLN("Opening door...");
    openDoor();
    isDoorOpen = 1; // Update state
  } else {
    SERIAL_PRINTLN("No change needed.");
  }

  // Go to sleep to save power
  goToSleep();
}

void openDoor() {
  SERIAL_PRINTLN("Motor moving to OPEN state");
  // Set direction for opening (e.g., IN1=HIGH, IN2=LOW)
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, LOW);
  // Set motor to full speed
  analogWrite(MOTOR_ENA, 255);
  delay(MOTOR_RUN_TIME_OPEN); // Run motor for specified time
  stopMotor();
}

void closeDoor() {
  SERIAL_PRINTLN("Motor moving to CLOSE state");
  // Set direction for closing (e.g., IN1=LOW, IN2=HIGH)
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, HIGH);
  // Set motor to full speed
  analogWrite(MOTOR_ENA, 255);
  delay(MOTOR_RUN_TIME_CLOSE); // Run motor for specified time
  stopMotor();
}

void stopMotor() {
  SERIAL_PRINTLN("Stopping motor.");
  // Disable the motor driver to stop the motor
  analogWrite(MOTOR_ENA, 0);
  // Set direction pins to low as a safe default state
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
}

void goToSleep() {
  // Save the current door state to RTC memory before sleeping
  uint32_t rtcData = isDoorOpen;
  ESP.rtcUserMemoryWrite(0, &rtcData, sizeof(rtcData));

  SERIAL_PRINT("Going to sleep for ");
  SERIAL_PRINT(SLEEP_INTERVAL_MIN);
  SERIAL_PRINTLN(" minutes...");
  
  // Turn off the Wi-Fi radio before sleeping to save power
  WiFi.forceSleepBegin();
  // ESP.deepSleep takes microseconds, so convert minutes to microseconds
  // WAKE_RF_DISABLED keeps the radio off when waking up
  ESP.deepSleep(SLEEP_INTERVAL_MIN * 60 * 1000000, WAKE_RF_DISABLED);
}

// The loop() function is not used because the ESP8266 will deep sleep
// and reset, running setup() again on each wake-up.
void loop() {
  // Intentionally empty
}