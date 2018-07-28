#include <LowPower.h>
#include <NewPing.h>

#define WIP 1
#define TRIGGER_PIN 11
#define ECHO_PIN 12
#define MIN_DISTANCE 10
#define MAX_DISTANCE 100
#define MIN_FREQUENCY 1
#define MAX_FREQUENCY 20
#define LED_PIN_MIN 2 // RED (PS: Don't use Serial ports)
#define LED_PIN_MAX 9 // GREEN
#define INACTIVITY_LIMIT 20

#define LED_COUNT (LED_PIN_MAX - LED_PIN_MIN + 1) // Inclusive the LED_PIN_MAX
#define DS ((MAX_DISTANCE - MIN_DISTANCE) / LED_COUNT)
#define INVALID_DISTANCE 0

NewPing
    g_sonarSensor(TRIGGER_PIN, ECHO_PIN,
                  MAX_DISTANCE); // NewPing setup of pins and maximum distance.
bool g_ledStates[LED_COUNT];
int g_inactivityCount = 0;
int g_lastMeasurement = 0;

int indexToPin(int index) { return LED_PIN_MIN + index; }

void setLedStates(bool state, int upto = LED_COUNT) {
  upto = min(upto, LED_COUNT);
  for (int i = 0; i < upto; ++i) {
    g_ledStates[i] = state;
  }
}

void writeLedPins() {
  for (int i = 0; i < LED_COUNT; ++i) {
    digitalWrite(indexToPin(i), g_ledStates[i]);
  }
}

void blink(int frequency = 10) {
  setLedStates(LOW);
  writeLedPins();
  delay(1000 / frequency);
  setLedStates(HIGH);
  writeLedPins();
  delay(1000 / frequency);
}

void roll(int frequency = 15) {
  for (int i = LED_COUNT - 1; i >= 0; --i) {
    setLedStates(LOW);
    g_ledStates[i] = HIGH;
    writeLedPins();
    delay(1000 / frequency);
  }
  for (int i = 0; i < LED_COUNT; ++i) {
    setLedStates(LOW);
    g_ledStates[i] = HIGH;
    writeLedPins();
    delay(1000 / frequency);
  }
}
void setupLedPins() {
  for (int i = 0; i < LED_COUNT; ++i) {
    pinMode(indexToPin(i), OUTPUT);
  }
}

void setupSerial() {
#ifdef WIP
  Serial.begin(9600);
#endif
}
void setup() {
  setupSerial();
  setupLedPins();
  setLedStates(LOW);
  writeLedPins();
  roll();
}

int getDistanceInCm() {
  auto uS = g_sonarSensor.ping_median();
  auto cm = g_sonarSensor.convert_cm(uS);
  return cm;
}

void catchInvalidMeasurement(int &cm) {
  if (cm == INVALID_DISTANCE || cm > MAX_DISTANCE) {
    cm = MAX_DISTANCE; // Make it more robust and improve visualisation in plot
  }
}

void visualizeAsLightbar(const int cm) {
  setLedStates(HIGH);
  if (cm > MIN_DISTANCE) {
    int numLedsOff = (cm - MIN_DISTANCE) / DS;
    setLedStates(LOW, numLedsOff);
    writeLedPins();
  } else {
    blink(20);
  }
}

void visualizeAsStrobe(const int cm) {

  if (cm > MIN_DISTANCE) {
    auto factor = (cm - MIN_DISTANCE) / (MAX_DISTANCE - MIN_DISTANCE);
    int frequency = MAX_FREQUENCY - (MAX_FREQUENCY - MIN_FREQUENCY) * factor;
    blink(frequency);
  } else {
    setLedStates(HIGH);
    writeLedPins();
  }
}

void goToSleep() {
#ifdef WIP
  Serial.println("Going to sleep.");
  Serial.flush();
#endif
  blink(15);
  setLedStates(LOW);
  writeLedPins();
  LowPower.powerDown(SLEEP_4S, ADC_OFF, BOD_OFF);
  // LowPower.idle(SLEEP_8S, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_OFF,
  // SPI_OFF, USART0_OFF, TWI_OFF);
}

void countInactivity(int distance) {
  if (distance == g_lastMeasurement) {
    ++g_inactivityCount;
  } else {
    g_inactivityCount = 0;
  }
  g_lastMeasurement = distance;
}

void sleepUponInactivity() {
  if (g_inactivityCount >= INACTIVITY_LIMIT) {
    // Do not reset counter, so that we go back to sleep quickly if measurement
    // stays the same
    goToSleep();
  }
}
void loop() {
  int distance = getDistanceInCm();
  catchInvalidMeasurement(distance);
  countInactivity(distance);
  sleepUponInactivity();
  visualizeAsLightbar(distance);
  // visualizeAsStrobe(distance);
#ifdef WIP
  Serial.println(distance);
#endif
}
