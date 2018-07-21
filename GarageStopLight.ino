#include <NewPing.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

#define TRIGGER_PIN 11
#define ECHO_PIN 12
#define MIN_DISTANCE 50
#define MAX_DISTANCE 200
#define MIN_FREQUENCY 1
#define MAX_FREQUENCY 20
#define LED_PIN_MIN 2 // RED (PS: Don't use Serial ports)
#define LED_PIN_MAX 9 // GREEN
#define INVALID_SLEEP_LIMIT 10
#define LED_COUNT (LED_PIN_MAX - LED_PIN_MIN + 1) // Inclusive the LED_PIN_MAX
#define DS ((MAX_DISTANCE - MIN_DISTANCE) / LED_COUNT)
#define INVALID_DISTANCE 0

NewPing
    g_sonarSensor(TRIGGER_PIN, ECHO_PIN,
                  MAX_DISTANCE); // NewPing setup of pins and maximum distance.
bool g_ledStates[LED_COUNT];
volatile int g_invalidCount = 0;

/// Watchdog timer Interrupt Service Routine
ISR(WDT_vect) {
  if (g_invalidCount == INVALID_SLEEP_LIMIT) {
    g_invalidCount = 0;
  } else {
    // Serial.println("WDT Overrun Error!");
  }
}

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

void setupWatchdogTimer() {
  MCUSR &= ~(1 << WDRF); /* WDT reset flag loeschen */
  WDTCSR |=
      (1 << WDCE) | (1 << WDE);   /* WDCE setzen, Zugriff auf Prescaler etc. */
  WDTCSR = 1 << WDP0 | 1 << WDP3; /* Prescaler auf 8.0 s */
  WDTCSR |= 1 << WDIE;            /* WDT Interrupt freigeben */
}

void setup() {
  setupLedPins();
  setupWatchdogTimer();

  setLedStates(LOW);
  writeLedPins();

  Serial.begin(9600);

  roll();
}

int getDistanceInCm() {
  auto uS = g_sonarSensor.ping_median();
  auto cm = g_sonarSensor.convert_cm(uS);
  return cm;
}

void catchInvalidMeasurement(int &cm) {
  if (cm == INVALID_DISTANCE || cm > MAX_DISTANCE) {
    ++g_invalidCount;
    cm = MAX_DISTANCE; // Make it more robust and improve visualisation in plot
  } else {
    g_invalidCount = 0; // One sane measurement heales
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
  digitalWrite(LED_PIN_MIN, HIGH);

  // Taken from
  // http://www.netzmafia.de/skripten/hardware/Arduino/Sleep/index.html
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  power_adc_disable();    /* Analog-Eingaenge abschalten */
  power_spi_disable();    /* SPI abschalten */
  power_timer0_disable(); /* Timer0 abschalten */
  power_timer2_disable(); /* Timer0 abschalten */
  power_twi_disable();    /* TWI abschalten */
  sleep_mode();
  sleep_disable();
  power_all_enable(); /* Komponenten wieder aktivieren */
}

void loop() {
  int distance = getDistanceInCm();
  catchInvalidMeasurement(distance);

  if (g_invalidCount == INVALID_SLEEP_LIMIT) {
    Serial.println("Going to sleep.");
    Serial.flush();
    goToSleep();
  } else {
    Serial.println(distance);
    visualizeAsLightbar(distance);
    // visualizeAsStrobe(distance);
  }
}
