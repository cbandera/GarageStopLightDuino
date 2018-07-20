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
#define INVALID 0

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
bool led_states[LED_COUNT];

int getPinId(int i) {
  return LED_PIN_MIN + i;
}

void resetLedStates() {
  for (int i = 0; i < LED_COUNT; ++i)
  {
    led_states[i] = LOW;
  }
}

void writeLedStates() {
  for (int i = 0; i < LED_COUNT; ++i) {
    digitalWrite(getPinId(i), led_states[i]);
  }
}

void setup() {
  for (int i = 0; i < LED_COUNT; ++i) {
    pinMode(getPinId(i), OUTPUT);
  }

  resetLedStates();
  writeLedStates();

  Serial.begin(9600);
}

void loop() {
  resetLedStates();

  unsigned long uS = sonar.ping_median();
  unsigned int cm = sonar.convert_cm(uS);

  if (cm == INVALID || cm > MAX_DISTANCE) {
    cm = MAX_DISTANCE; // Make it more robust and improve visualisation in plot
  }
  Serial.println(cm);

  for (int i = 0; i < LED_COUNT; ++i)
  {
    if (cm < MIN_DISTANCE + DS * i )
      led_states[i] = HIGH;
  }

  writeLedStates();

  delay(DELAY_IN_MS);
}
