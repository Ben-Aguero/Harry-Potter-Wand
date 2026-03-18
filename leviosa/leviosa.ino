#include <Arduino.h>

// --- PIN DEFINITIONS ---
const int ENA_PIN   = 25;
const int IN1_PIN   = 26;
const int IN2_PIN   = 27;
const int ENCODER_A = 32;
const int ENCODER_B = 33;

// --- PWM CONFIG ---
const int PWM_FREQ       = 30000;
const int PWM_RESOLUTION = 8;

// --- PID TUNING (adjust these for your motor) ---
const float Kp = 2.0f;
const float Ki = 0.03f;

// --- MOTION LIMITS ---
const int MIN_PWM_UP   = 65;
const int MAX_PWM_UP   = 155;
const int MIN_PWM_DOWN = 22;
const int MAX_PWM_DOWN = 75;
const int DEADBAND     = 5;

// --- ENCODER STATE ---
volatile long currentPosition  = 0;
volatile unsigned long lastISRTime = 0;

void IRAM_ATTR encoderISR() {
  unsigned long now = micros();
  if (now - lastISRTime > 500) {
    currentPosition += (digitalRead(ENCODER_B) > 0) ? 1 : -1;
  }
  lastISRTime = now;
}

// ─── HELPERS ──────────────────────────────────────────────────────────────────

void motorStop() {
  ledcWrite(ENA_PIN, 0);
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
}

// Drives the motor based on the sign of `error` and a raw PWM magnitude.
// Applies direction-appropriate min/max clamping.
void motorDrive(long error, int rawPWM) {
  int pwr = abs(rawPWM);
  if (error > 0) {
    pwr = constrain(pwr, MIN_PWM_UP, MAX_PWM_UP);
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
    ledcWrite(ENA_PIN, pwr);
  } else {
    pwr = constrain(pwr, MIN_PWM_DOWN, MAX_PWM_DOWN);
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);
    ledcWrite(ENA_PIN, pwr);
  }
}

// ─── PI POSITION HOLD ─────────────────────────────────────────────────────────
// FIX: Motor now STOPS inside the deadband instead of always driving.
// FIX: Safety timeout prevents infinite loops on stall/encoder glitch.
bool moveAndHold(long target, int holdMs, bool cutPower, unsigned long timeoutMs = 5000) {
  long errorSum   = 0;
  long prevError  = 0;
  bool reached    = false;
  unsigned long reachedAt = 0;
  unsigned long startedAt = millis();

  while (true) {

    // ── Safety timeout ──────────────────────────────────────────────────────
    if ((millis() - startedAt) > timeoutMs) {
      Serial.println("WARN: moveAndHold timeout!");
      motorStop();
      return false;
    }

    long pos   = currentPosition;   // one atomic snapshot of the volatile
    long error = target - pos;

    // ── CRITICAL FIX: Inside deadband → stop motor and run hold timer ───────
    if (abs(error) <= DEADBAND) {
      motorStop();
      errorSum = 0;                 // clear integral on arrival
      if (!reached) {
        reached   = true;
        reachedAt = millis();
      }
      if ((millis() - reachedAt) >= (unsigned long)holdMs) {
        if (cutPower) motorStop();
        return true;
      }
      delay(2);
      continue;
    }

    // ── If we drifted back out of deadband, restart the hold timer ──────────
    reached = false;

    // ── Anti-windup: only integrate when close, reset on zero-crossing ──────
    if (abs(error) < 60) {
      if ((error > 0) != (prevError > 0)) errorSum = 0;   // zero-crossing
      errorSum = constrain(errorSum + error, -3000L, 3000L);
    } else {
      errorSum = 0;
    }
    prevError = error;

    int rawPWM = (int)((error * Kp) + (errorSum * Ki));
    motorDrive(error, rawPWM);
    delay(2);
  }
}

// ─── SMOOTH GLIDE ─────────────────────────────────────────────────────────────
// FIX: Uses actual currentPosition as start (not an assumed value).
// FIX: Settle timeout exits the loop if the motor can't reach the final target.
bool glideToPosition(long endPos, unsigned long durationMs) {
  long startPos = currentPosition;           // capture real start
  long errorSum = 0;
  long prevError = 0;
  unsigned long startTime = millis();
  const unsigned long SETTLE_TIMEOUT = 2000; // extra ms to settle after ramp ends

  Serial.print("Glide "); Serial.print(startPos);
  Serial.print(" -> "); Serial.println(endPos);

  while (true) {
    unsigned long elapsed = millis() - startTime;

    // ── Compute moving target ─────────────────────────────────────────────
    long movingTarget;
    if (elapsed < durationMs) {
      float t = (float)elapsed / (float)durationMs;
      movingTarget = startPos + (long)((endPos - startPos) * t);
    } else {
      movingTarget = endPos;
      // After ramp ends: check settle or timeout
      if (abs(endPos - currentPosition) <= DEADBAND) {
        motorStop();
        return true;
      }
      if ((elapsed - durationMs) > SETTLE_TIMEOUT) {
        Serial.println("WARN: glide settle timeout!");
        motorStop();
        return false;
      }
    }

    long error = movingTarget - currentPosition;

    if (abs(error) <= DEADBAND) {            // coast through small errors
      motorStop();
      delay(2);
      continue;
    }

    if (abs(error) < 60) {
      if ((error > 0) != (prevError > 0)) errorSum = 0;
      errorSum = constrain(errorSum + error, -3000L, 3000L);
    } else {
      errorSum = 0;
    }
    prevError = error;

    int rawPWM = (int)((error * Kp) + (errorSum * Ki));
    motorDrive(error, rawPWM);
    delay(2);
  }
}

// ─── SETUP ────────────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);

  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  ledcAttach(ENA_PIN, PWM_FREQ, PWM_RESOLUTION);

  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, RISING);

  motorStop();
  currentPosition = 0;

  Serial.println("System ready. Starting in 3s...");
  delay(3000);
}

// ─── LEVIOSA SEQUENCE ─────────────────────────────────────────────────────────

void loop() {
  Serial.println("=== Leviosa START ===");

  // Rise and hang
  moveAndHold(190, 300, false, 6000);

  // Bob twice
  for (int i = 0; i < 2; i++) {
    glideToPosition(100, 800);
    glideToPosition(170, 800);
  }

  // Smooth descent back to rest
  glideToPosition(0, 3000);

  motorStop();
  Serial.println("=== Leviosa COMPLETE. Resting 8s ===");
  delay(8000);
}
