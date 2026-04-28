/*
  Arduino Uno:
  D2  -> output 30 Hz TTL pulse
  A0  -> input TTL monitor
  Serial -> send TTL event when A0 rising edge is detected

  Serial format:
  TTL,<index>,<timestamp_us>

  Example:
  TTL,1,1234567
  TTL,2,1267899
*/

const uint8_t TTL_OUT_PIN = 2;   // D2 output
const uint8_t TTL_IN_PIN  = A0;  // A0 input monitor

const unsigned long PERIOD_US = 33333UL;      // 30 Hz
const unsigned long PULSE_WIDTH_US = 2000UL;  // 2 ms pulse width

unsigned long nextPulseTime = 0;
unsigned long pulseStartTime = 0;

bool pulseHigh = false;

// edge detect
int lastInputState = LOW;
unsigned long ttlIndex = 0;

void setup() {
  pinMode(TTL_OUT_PIN, OUTPUT);
  digitalWrite(TTL_OUT_PIN, LOW);

  pinMode(TTL_IN_PIN, INPUT);

  Serial.begin(115200);
  delay(300);

  unsigned long nowUs = micros();
  nextPulseTime = nowUs + PERIOD_US;

  Serial.println("ARDUINO_TTL_SIM_START");
}

void loop() {
  unsigned long nowUs = micros();

  // =========================
  // Part 1: generate 30 Hz TTL on D2
  // =========================
  if (!pulseHigh && (long)(nowUs - nextPulseTime) >= 0) {
    digitalWrite(TTL_OUT_PIN, HIGH);
    pulseHigh = true;
    pulseStartTime = nowUs;

    nextPulseTime += PERIOD_US;
  }

  if (pulseHigh && (long)(nowUs - pulseStartTime) >= (long)PULSE_WIDTH_US) {
    digitalWrite(TTL_OUT_PIN, LOW);
    pulseHigh = false;
  }

  // =========================
  // Part 2: monitor A0 input
  // rising edge detect
  // =========================
  int currentInputState = digitalRead(TTL_IN_PIN);

  if (currentInputState == HIGH && lastInputState == LOW) {
    ttlIndex++;

    unsigned long eventTimeUs = micros();

    Serial.print("TTL,");
    Serial.print(ttlIndex);
    Serial.print(",");
    Serial.println(eventTimeUs);
  }

  lastInputState = currentInputState;
}