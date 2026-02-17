#include <Arduino.h>

#define ticksToUs(_ticks) ((double) _ticks / 2.0) // (prescaled ticks * 8) / (16 MHz) * (1000000 us/s)
#define usToTicks(_us) ((uint16_t) (_us * 2))

// Full forward value: 1.861ms
// Full neutral value: 1.469ms
// Full reverse value: 1.077ms

constexpr uint8_t LINK_LIVE_PIN = 20; // MAKE SURE TO SET TO CORRECT VALUE
constexpr uint16_t NEUTRAL_TICKS = usToTicks(1469); // Neutral pulse length
constexpr uint16_t MAX_TICKS = usToTicks(1861); // Max pulse length
constexpr uint16_t MAIN_KS = 0; // Ticks
constexpr uint16_t MAIN_KV = 0.1; // Ticks per RPM

static bool inited = false;

static volatile bool runDataTick = false;
static volatile uint8_t currentTick = 0;
static volatile bool ticked = false;

static volatile uint16_t rawDataItems[11];

static uint16_t targetThrottleTicks = usToTicks(1469); // Neutral pulse length: 1.469ms

long prevTime = 0; // Serial delay tracking

SIGNAL (TIMER4_COMPA_vect) {
    // Serial.println("P");
    detachInterrupt(digitalPinToInterrupt(LINK_LIVE_PIN));
    pinMode(LINK_LIVE_PIN, OUTPUT);
    digitalWrite(LINK_LIVE_PIN, LOW);
    // Serial.println("T");

    TCNT4 = 0;
    OCR4B = targetThrottleTicks;

    if (ticked == false) {
        currentTick = 0;
    } else {
        ticked = false;
    }
}

void dataTick() {
    if (!runDataTick) {
        runDataTick = true;
        return;
    }
    detachInterrupt(digitalPinToInterrupt(LINK_LIVE_PIN));
    rawDataItems[currentTick] = TCNT4 - OCR4B;
    // if (currentTick == 6) Serial.println(rawDataItems[currentTick]);
    // Serial.println(currentTick);
    ticked = true;
    currentTick++;
}

SIGNAL (TIMER4_COMPB_vect) {
    digitalWrite(LINK_LIVE_PIN, HIGH);
    pinMode(LINK_LIVE_PIN, INPUT);
    // EIFR = 0; // &= 0b11111101; // MAKE SURE TO CHANGE THIS IF NOT USING PIN 20
    runDataTick = false;
    attachInterrupt(digitalPinToInterrupt(LINK_LIVE_PIN), dataTick, FALLING);
}

float latestRPM() {
    return
        (ticksToUs(rawDataItems[6])
        * (500.0 / (double) (ticksToUs(rawDataItems[0]) - ticksToUs(min(rawDataItems[9], rawDataItems[10])))) // Applying the timescale calibration
        - 500.0); // The minimum signal is 0.5ms, meaning a 0.5ms data point after calibration represents 0
        // * (20416.7 / 1000.0); // For RPM specifically, 1ms represents 20416.7 RPM => (20416.7 RPM / 1000us)
}

void setup() {
    Serial.begin(115200);
    
    Serial.println("[SERIAL STARTED]");

    TCCR4A = 0;
    TCCR4B = 2; // 00 00 00 10
    TCNT4 = 0;
    OCR4A = 40000;
    OCR4B = NEUTRAL_TICKS;
    TIFR4 = 6; // 00 00 01 10
    TIMSK4 = 6; // 00 00 01 10

    Serial.println("[TIMER SETUP]");

    inited = true;
}

void loop() {
  // Serial.println(currentTick);

  unsigned long currentTime = millis();

  if (currentTime - prevTime >= 1000) {
    prevTime = currentTime;

    float rpm = latestRPM();

    Serial.println(rpm);
    // Serial.println(rawDataItems[6]);
    // Serial.println(rawDataItems[0]);
    // Serial.println((500.0 / (double) (ticksToUs(rawDataItems[0]) - ticksToUs(min(rawDataItems[9], rawDataItems[10])))));
  }

  int bytes = Serial.available();

  if (bytes > 0) {
    char digits[bytes];

    for (int i = 0; i < bytes; i++) {
      digits[i] = Serial.read();
    }

    digits[bytes - 1] = 0;

    int targetRPM = constrain(atoi(digits), 0, 30000);

    /// Serial.println(targetRPM);

    uint16_t feedforward = 60; // (MAIN_KS * (targetRPM == 0 ? 0 : 1)) + (MAIN_KV * targetRPM);
    targetThrottleTicks = constrain(NEUTRAL_TICKS + feedforward, NEUTRAL_TICKS, MAX_TICKS);
  }
}