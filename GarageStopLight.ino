#include <NewPing.h>

#define DELAY_IN_MS 100
#define TRIGGER_PIN 11
#define ECHO_PIN 12
#define MAX_DISTANCE 200
#define MIN_DISTANCE 50
#define LED_PIN_MIN 2 // RED
#define LED_PIN_MAX 7 // GREEN

#define LED_COUNT (LED_PIN_MAX - LED_PIN_MIN + 1) // Inclusive the LED_PIN_MAX
#define DS ( (MAX_DISTANCE - MIN_DISTANCE) / LED_COUNT )
#define INVALID_DISTANCE 0

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
bool led_states[LED_COUNT];

int indexToPin(int index) {
  return LED_PIN_MIN + index;
}

void resetLedStates() {
  for (int i = 0; i < LED_COUNT; ++i)
  {
    led_states[i] = LOW;
  }
}

void writeLedStates() {
  for (int i = 0; i < LED_COUNT; ++i) {
    digitalWrite(indexToPin(i), led_states[i]);
  }
}

unsigned int getDistanceInCm() {
  auto uS = sonar.ping_median();
  auto cm = sonar.convert_cm(uS);

  if (cm == INVALID_DISTANCE || cm > MAX_DISTANCE) {
    cm = MAX_DISTANCE; // Make it more robust and improve visualisation in plot
  }
  
  return cm;
}

void setLedStatesBasedOnDistance(unsigned int cm)
{
  for (int i = 0; i < LED_COUNT; ++i)
  {
    if (cm < MIN_DISTANCE + DS * i )
      led_states[i] = HIGH;
  }
}

void setup() {
  for (int i = 0; i < LED_COUNT; ++i) {
    pinMode(indexToPin(i), OUTPUT);
  }

  resetLedStates();
  writeLedStates();

  Serial.begin(9600);
}

void loop() {
  resetLedStates();

  auto distance = getDistanceInCm();
  Serial.println(distance);

  setLedStatesBasedOnDistance(distance);
  writeLedStates();

  delay(DELAY_IN_MS);
}
