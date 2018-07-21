#include <NewPing.h>

#define DELAY_IN_MS 100
#define TRIGGER_PIN 11
#define ECHO_PIN 12
#define MIN_DISTANCE 50
#define MAX_DISTANCE 200
#define LED_PIN_MIN 2 // RED (PS: Don't use Serial ports)
#define LED_PIN_MAX 9 // GREEN

#define LED_COUNT (LED_PIN_MAX - LED_PIN_MIN + 1) // Inclusive the LED_PIN_MAX
#define DS ((MAX_DISTANCE - MIN_DISTANCE) / LED_COUNT)
#define INVALID_DISTANCE 0

NewPing g_sonarSensor(TRIGGER_PIN, ECHO_PIN,
              MAX_DISTANCE); // NewPing setup of pins and maximum distance.
bool g_ledStates[LED_COUNT];

int indexToPin(int index) { return LED_PIN_MIN + index; }

void setLedStates(bool state, int upto = LED_COUNT)
{
  upto = min(upto, LED_COUNT);
  for (int i = 0; i < upto; ++i)
  {
    g_ledStates[i] = state;
  }
}

void writeLedPins()
{
  for (int i = 0; i < LED_COUNT; ++i)
  {
    digitalWrite(indexToPin(i), g_ledStates[i]);
  }
}

void blink()
{
  setLedStates(LOW);
  writeLedPins();
  delay(DELAY_IN_MS);
  setLedStates(HIGH);
  writeLedPins();
  // Don't delay here, as the loop will delay anyway
}

unsigned int getDistanceInCm()
{
  auto uS = g_sonarSensor.ping_median();
  auto cm = g_sonarSensor.convert_cm(uS);

  if (cm == INVALID_DISTANCE || cm > MAX_DISTANCE)
  {
    cm = MAX_DISTANCE; // Make it more robust and improve visualisation in plot
  }

  return cm;
}

void visualizeDistance(int cm)
{
  setLedStates(HIGH);
  int numLedsOff = (cm - MIN_DISTANCE) / DS;
  if (numLedsOff >= 0)
  {
    setLedStates(LOW, numLedsOff);
    writeLedPins();
  }
  else
  {
    blink();
  }
}

void setup()
{
  for (int i = 0; i < LED_COUNT; ++i)
  {
    pinMode(indexToPin(i), OUTPUT);
  }

  setLedStates(LOW);
  writeLedPins();

  Serial.begin(9600);

  blink();
  blink();
  blink();
}

void loop()
{
  auto distance = getDistanceInCm();
  Serial.println(distance);
  visualizeDistance(distance);
  delay(DELAY_IN_MS);
}
