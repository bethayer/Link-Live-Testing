#include <Arduino.h>

#define ticksToUs(_ticks) ((double) _ticks / 2.0) // (prescaled ticks * 8) / (16 MHz) * (1000000 us/s)
#define usToTicks(_us) ((uint16_t) (_us * 2))

constexpr uint8_t LINK_LIVE_PIN = 0;
constexpr uint16_t NEUTRAL_TICKS = 3000; // 1.5ms (CHECK IF THIS IS CORRECT)
constexpr uint16_t MAIN_KS = 0; // Ticks
constexpr uint16_t MAIN_KV = 0; // Ticks per RPM

static bool inited = false;

static volatile uint8_t currentTick = 0;
static volatile bool ticked = false;

static volatile uint16_t rawDataItems[11] {};

static uint16_t targetThrottleTicks = 3000;

long prevTime = 0; // Serial delay tracking

SIGNAL (TIMER4_COMPA_vect) {
    pinMode(LINK_LIVE_PIN, OUTPUT);
    digitalWrite(LINK_LIVE_PIN, LOW);

    TCNT4 = 0;
    OCR4B = targetThrottleTicks;

    if (ticked == false) {
        currentTick = 0;
    } else {
        ticked = false;
    }
}

void dataTick() {
    detachInterrupt(digitalPinToInterrupt(LINK_LIVE_PIN));
    rawDataItems[currentTick] = TCNT4 - OCR4A;
    ticked = true;
    currentTick++;
}

SIGNAL (TIMER4_COMPB_vect) {
    digitalWrite(LINK_LIVE_PIN, HIGH);
    pinMode(LINK_LIVE_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(LINK_LIVE_PIN), dataTick, FALLING);
}

float latestRPM() {
    return
        (ticksToUs(rawDataItems[6])
        * (500.0 / (double) (rawDataItems[0] - min(rawDataItems[9], rawDataItems[10]))) // Applying the timescale calibration
        - 500.0) // The minimum signal is 0.5ms, meaning a 0.5ms data point after calibration represents 0
        * (20416.7 / 1000.0); // For RPM specifically, 1ms represents 20416.7 RPM => (20416.7 RPM / 1000us)
}

void setup() {
    Serial.begin(115200);

    TCCR4A = 0;
    TCCR4B = 2; // 00 00 00 10
    TCNT4 = 0;
    OCR4A = 40000;
    OCR4B = NEUTRAL_TICKS;
    TIFR4 = 6; // 00 00 01 10
    TIMSK4 = 6; // 00 00 01 10

    inited = true;
}

void loop() {
  unsigned long currentTime = millis();

  if (currentTime - prevTime >= 1000) {
    prevTime = currentTime;

    float rpm = latestRPM();

    Serial.println(rpm);
  }

  int bytes = Serial.available();

  if (bytes > 0) {
    char digits[bytes];

    for (int i = 0; i < bytes; i++) {
      digits[i] = Serial.read();
    }

    int targetRPM = constrain(atoi(digits), 0, 30000);

    uint16_t feedforward = (MAIN_KS * (targetRPM == 0 ? 0 : 1)) + (MAIN_KV * targetRPM);
    targetThrottleTicks = NEUTRAL_TICKS + feedforward;
  }
}