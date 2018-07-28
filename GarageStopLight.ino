#include <LowPower.h>
#include <NewPing.h>

/// Behavioral Options
#define USE_SERIAL 0
#define visualizeFct visualizeAsLightbar // visualizeAsStrobe
#define SLEEP_DURATION SLEEP_4S
#define MIN_DISTANCE 10
#define MAX_DISTANCE 100
#define MIN_STROBE_FREQUENCY 1
#define MAX_STROBE_FREQUENCY 20
#define INACTIVITY_LIMIT 50

/// Pin Setup
#define TRIGGER_PIN 11
#define ECHO_PIN 12
#define LED_PIN_MIN 2 // RED (PS: Don't use Serial ports)
#define LED_PIN_MAX 9 // GREEN

/// Constants
#define LED_COUNT (LED_PIN_MAX - LED_PIN_MIN + 1) // Inclusive the LED_PIN_MAX
#define DS ((MAX_DISTANCE - MIN_DISTANCE) / LED_COUNT)
#define INVALID_DISTANCE 0

/// Globals
NewPing
    g_sonarSensor(TRIGGER_PIN, ECHO_PIN,
                  MAX_DISTANCE); // NewPing setup of pins and maximum distance.
bool g_ledStates[LED_COUNT];
int g_inactivityCount = 0;
int g_lastMeasurement = 0;

int indexToPin(int index) { return LED_PIN_MIN + index; }

void setAllLedStates(bool state, int upto = LED_COUNT) {
  upto = min(upto, LED_COUNT);
  for (int i = 0; i < upto; ++i) {
    g_ledStates[i] = state;
  }
}

void writeLedStatesToPins() {
  for (int i = 0; i < LED_COUNT; ++i) {
    digitalWrite(indexToPin(i), g_ledStates[i]);
  }
}

void blinkLeds(int frequency = 10) {
  setAllLedStates(LOW);
  writeLedStatesToPins();
  delay(1000 / frequency);
  setAllLedStates(HIGH);
  writeLedStatesToPins();
  delay(1000 / frequency);
}

void rollLeds(int frequency = 15) {
  for (int i = LED_COUNT - 1; i >= 0; --i) {
    setAllLedStates(LOW);
    g_ledStates[i] = HIGH;
    writeLedStatesToPins();
    delay(1000 / frequency);
  }
  for (int i = 0; i < LED_COUNT; ++i) {
    setAllLedStates(LOW);
    g_ledStates[i] = HIGH;
    writeLedStatesToPins();
    delay(1000 / frequency);
  }
}
void setupLedPins() {
  for (int i = 0; i < LED_COUNT; ++i) {
    pinMode(indexToPin(i), OUTPUT);
  }
}

void setupSerial() {
#ifdef USE_SERIAL
  Serial.begin(9600);
#endif
}
void setup() {
  setupSerial();
  setupLedPins();
  setAllLedStates(LOW);
  writeLedStatesToPins();
  rollLeds();
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
  setAllLedStates(HIGH);
  if (cm > MIN_DISTANCE) {
    int uptoLedNr = (cm - MIN_DISTANCE) / DS;
    setAllLedStates(LOW, uptoLedNr);
    writeLedStatesToPins();
  } else {
    blinkLeds(20);
  }
}

void visualizeAsStrobe(const int cm) {
  if (cm > MIN_DISTANCE) {
    auto factor = (cm - MIN_DISTANCE) / (MAX_DISTANCE - MIN_DISTANCE);
    int frequency = MAX_STROBE_FREQUENCY -
                    (MAX_STROBE_FREQUENCY - MIN_STROBE_FREQUENCY) * factor;
    blinkLeds(frequency);
  } else {
    setAllLedStates(HIGH);
    writeLedStatesToPins();
  }
}

void goToSleep() {
#ifdef USE_SERIAL
  Serial.println("Going to sleep.");
  Serial.flush();
#endif
  setAllLedStates(LOW);
  writeLedStatesToPins();
  LowPower.powerDown(SLEEP_DURATION, ADC_OFF, BOD_OFF);
  // LowPower.idle(SLEEP_DURATION, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_OFF,
  // SPI_OFF, USART0_OFF, TWI_OFF);
}

void countInactivity(int distance) {
  if (abs(distance - g_lastMeasurement) <= 1) {
    ++g_inactivityCount;
  } else {
    g_inactivityCount = 0;
    g_lastMeasurement = distance;
  }
}

bool sleepUponInactivity() {
  if (g_inactivityCount >= INACTIVITY_LIMIT) {
    // Do not reset counter, so that we go back to sleep quickly if measurement
    // stays the same
    goToSleep();
    return true;
  }
  return false;
}
void loop() {
  int distance = getDistanceInCm();
  catchInvalidMeasurement(distance);
  countInactivity(distance);
#ifdef USE_SERIAL
  Serial.print(distance);
  Serial.print(" ");
  Serial.print(MIN_DISTANCE);
  Serial.print(" ");
  Serial.print(MAX_DISTANCE);
  Serial.print(" ");
  Serial.print(g_inactivityCount);
  Serial.print(" ");
  Serial.println(INACTIVITY_LIMIT);
  
#endif
  if (sleepUponInactivity())
    return; // aka continue
  else
    visualizeFct(distance);
}
