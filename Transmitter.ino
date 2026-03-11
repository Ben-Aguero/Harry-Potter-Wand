#define OUTPUT_PIN 9
#define HALF_PERIOD_US 125  // 4kHz = 250µs period, so half-period = 125µs

void setup() {
  pinMode(OUTPUT_PIN, OUTPUT);
}

void loop() {
  digitalWrite(OUTPUT_PIN, HIGH);
  delayMicroseconds(HALF_PERIOD_US);
  digitalWrite(OUTPUT_PIN, LOW);
  delayMicroseconds(HALF_PERIOD_US);
}
