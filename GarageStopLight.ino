#include <NewPing.h>

#define DELAY_IN_MS 100
#define TRIGGER_PIN 11
#define ECHO_PIN 12
#define MAX_DISTANCE 200
#define MIN_DISTANCE 50
#define LED_PIN_MIN 2 // RED (Don't use Serial ports)
#define LED_PIN_MAX 9 // GREEN

#define LED_COUNT (LED_PIN_MAX - LED_PIN_MIN + 1) // Inclusive the LED_PIN_MAX
#define DS ((MAX_DISTANCE - MIN_DISTANCE) / LED_COUNT)
#define INVALID_DISTANCE 0

NewPing sonar(TRIGGER_PIN, ECHO_PIN,
              MAX_DISTANCE); // NewPing setup of pins and maximum distance.
bool led_states[LED_COUNT];

int indexToPin(int index) { return LED_PIN_MIN + index; }

void setLedStates(bool state, int upto = LED_COUNT) {
  for (int i = 0; i < upto; ++i) {
    led_states[i] = state;
  }
}

void writeLedPins() {
  for (int i = 0; i < LED_COUNT; ++i) {
    digitalWrite(indexToPin(i), led_states[i]);
  }
}

void blink() {
  setLedStates(LOW);
  writeLedPins();
  delay(DELAY_IN_MS);
  setLedStates(HIGH);
  writeLedPins();
  delay(DELAY_IN_MS);
}

unsigned int getDistanceInCm() {
  auto uS = sonar.ping_median();
  auto cm = sonar.convert_cm(uS);

  if (cm == INVALID_DISTANCE || cm > MAX_DISTANCE) {
    cm = MAX_DISTANCE; // Make it more robust and improve visualisation in plot
  }

  return cm;
}

void visualizeDistance(unsigned int cm) {
  setLedStates(HIGH);

  auto numLedsOff = (cm - MIN_DISTANCE) % DS;
  if (numLedsOff >= 0) {
    setLedStates(LOW, numLedsOff);
  } else {
    blink();
  }

  writeLedPins();
}

void setup() {
  for (int i = 0; i < LED_COUNT; ++i) {
    pinMode(indexToPin(i), OUTPUT);
  }

  setLedStates(LOW);
  writeLedPins();

  Serial.begin(9600);

  blink();
  blink();
  blink();
}

void loop() {
  auto distance = getDistanceInCm();
  Serial.println(distance);

  visualizeDistance(distance);

  delay(DELAY_IN_MS);
}
