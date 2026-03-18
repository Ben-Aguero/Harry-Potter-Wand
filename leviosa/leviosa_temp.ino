#include "FFT.h"
#include "DFRobotDFPlayerMini.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include <ESP32Servo.h>

// ─── TODO ──────────────────────────────────────────────────────────────────────
// TODO: Define LEVIOSA_LED_PIN (or LED strip pin) for the leviosa levitation effect
// TODO: Define LUMOS_LED_PIN   (or LED strip pin) for the lumos light effect
// ───────────────────────────────────────────────────────────────────────────────


// ═══════════════════════════════════════════════════════════════════════════════
//  SECTION 1 — PIN DEFINITIONS
// ═══════════════════════════════════════════════════════════════════════════════

// --- Pixie / hit ---
#define ADC_PIN        15   // Wand signal ADC (pixie)
#define LED_PIN        14   // Debug LED
#define HIT_LED        12   // Hit indicator LED
#define VOLUME_PIN     34   // Volume potentiometer (left pot)
#define THRESHOLD_PIN  35   // Threshold potentiometer (right pot)

// --- Leviosa motor ---
#define ADC_PIN_LEVIOSA 36  // Second wand ADC for leviosa detection (adjust as needed)
#define ENA_PIN         25
#define IN1_PIN         26
#define IN2_PIN         27
#define ENCODER_A       32
#define ENCODER_B       33

// --- Servo ---
const int servoPin = 13;

// --- DFPlayer UART ---
#define RXD2 5
#define TXD2 4


// ═══════════════════════════════════════════════════════════════════════════════
//  SECTION 2 — FFT / SAMPLING CONFIG
// ═══════════════════════════════════════════════════════════════════════════════

#define SAMPLE_RATE     16000
#define CHECK_RATE      1
#define FFT_N           1024
#define TARGET_FREQ     4000.0
#define HIT_PROCESS_TIMER   5000    // ms cooldown after pixie hit
#define LEVIOSA_COOLDOWN_MS 20000   // ms cooldown after leviosa sequence (~full duration + rest)
#define MAX_ADC         4095
#define MAX_VOLUME      30
#define MAX_THRESHOLD   20000
#define MIN_THRESHOLD   100

// Servo travel
#define INITIAL_POSITION 85
#define FINAL_POSITION  175
#define MOVE_DELAY      10


// ═══════════════════════════════════════════════════════════════════════════════
//  SECTION 3 — LEVIOSA MOTOR / PID CONFIG
// ═══════════════════════════════════════════════════════════════════════════════

#define PWM_FREQ       30000
#define PWM_RESOLUTION 8

// PI tuning
const float Kp = 2.0f;
const float Ki = 0.03f;

// PWM travel limits
const int MIN_PWM_UP   = 65;
const int MAX_PWM_UP   = 155;
const int MIN_PWM_DOWN = 22;
const int MAX_PWM_DOWN = 75;
const int DEADBAND     = 5;

// Encoder state (volatile — written in ISR)
volatile long          currentPosition = 0;
volatile unsigned long lastISRTime     = 0;

void IRAM_ATTR encoderISR() {
  unsigned long now = micros();
  if (now - lastISRTime > 500) {
    currentPosition += (digitalRead(ENCODER_B) > 0) ? 1 : -1;
  }
  lastISRTime = now;
}


// ═══════════════════════════════════════════════════════════════════════════════
//  SECTION 4 — STATE MACHINES & GLOBALS
// ═══════════════════════════════════════════════════════════════════════════════

typedef enum { WAITING, PROCESSING } hit_state_e;
hit_state_e hit_state     = WAITING;
hit_state_e leviosa_state = WAITING;
hit_state_e lumos_state   = WAITING;

// Pixie FFT buffers
float fft_input[FFT_N];
float fft_output[FFT_N];
int   sampleIndex = 0;

// Leviosa FFT buffers (separate channel)
float fft_input_leviosa[FFT_N];
float fft_output_leviosa[FFT_N];
int   sampleIndex_leviosa = 0;

float volatile threshold = 750;

// Timing
unsigned long previousMicros       = 0;
unsigned long previousMicrosPot    = 0;
const long sampleInterval          = 1000000 / SAMPLE_RATE;
const long checkPotInterval        = 1000000 / CHECK_RATE;

// Peripheral objects
DFRobotDFPlayerMini myDFPlayer;
Servo myservo;
int pos = INITIAL_POSITION;

// FreeRTOS task handle so we can guard against double-launch
static TaskHandle_t leviosaTaskHandle = NULL;


// ═══════════════════════════════════════════════════════════════════════════════
//  SECTION 5 — MOTOR HELPERS
// ═══════════════════════════════════════════════════════════════════════════════

void motorStop() {
  ledcWrite(ENA_PIN, 0);
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
}

// Drive motor in the direction implied by error's sign, with per-direction PWM limits.
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


// ═══════════════════════════════════════════════════════════════════════════════
//  SECTION 6 — PI POSITION HOLD
// ═══════════════════════════════════════════════════════════════════════════════

bool moveAndHold(long target, int holdMs, bool cutPower, unsigned long timeoutMs = 5000) {
  long errorSum  = 0;
  long prevError = 0;
  bool reached   = false;
  unsigned long reachedAt = 0;
  unsigned long startedAt = millis();

  while (true) {
    if ((millis() - startedAt) > timeoutMs) {
      Serial.println("WARN: moveAndHold timeout!");
      motorStop();
      return false;
    }

    long pos_snap = currentPosition;
    long error    = target - pos_snap;

    if (abs(error) <= DEADBAND) {
      motorStop();
      errorSum = 0;
      if (!reached) { reached = true; reachedAt = millis(); }
      if ((millis() - reachedAt) >= (unsigned long)holdMs) {
        if (cutPower) motorStop();
        return true;
      }
      delay(2);
      continue;
    }

    reached = false;

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


// ═══════════════════════════════════════════════════════════════════════════════
//  SECTION 7 — SMOOTH GLIDE
// ═══════════════════════════════════════════════════════════════════════════════

bool glideToPosition(long endPos, unsigned long durationMs) {
  long startPos  = currentPosition;
  long errorSum  = 0;
  long prevError = 0;
  unsigned long startTime = millis();
  const unsigned long SETTLE_TIMEOUT = 2000;

  Serial.print("Glide "); Serial.print(startPos);
  Serial.print(" -> ");   Serial.println(endPos);

  while (true) {
    unsigned long elapsed = millis() - startTime;

    long movingTarget;
    if (elapsed < durationMs) {
      float t     = (float)elapsed / (float)durationMs;
      movingTarget = startPos + (long)((endPos - startPos) * t);
    } else {
      movingTarget = endPos;
      if (abs(endPos - currentPosition) <= DEADBAND) { motorStop(); return true; }
      if ((elapsed - durationMs) > SETTLE_TIMEOUT) {
        Serial.println("WARN: glide settle timeout!");
        motorStop();
        return false;
      }
    }

    long error = movingTarget - currentPosition;

    if (abs(error) <= DEADBAND) { motorStop(); delay(2); continue; }

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


// ═══════════════════════════════════════════════════════════════════════════════
//  SECTION 8 — LEVIOSA FREERTOS TASK
//  Runs the full Leviosa animation sequence on a separate core/stack so it
//  never blocks the FFT sampling loop.
// ═══════════════════════════════════════════════════════════════════════════════

void leviosaSequenceTask(void *pvParameters) {
  Serial.println("=== Leviosa START ===");

  // TODO: digitalWrite(LEVIOSA_LED_PIN, HIGH);  // turn on levitation LED

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

  // TODO: digitalWrite(LEVIOSA_LED_PIN, LOW);   // turn off levitation LED

  Serial.println("=== Leviosa COMPLETE — starting cooldown ===");

  // Start the cooldown timer so leviosa_state returns to WAITING after the
  // configured delay (prevents immediate re-trigger).
  leviosaProcessTimer_start();   // forward-declared below

  leviosaTaskHandle = NULL;
  vTaskDelete(NULL);  // task cleans itself up
}


// ═══════════════════════════════════════════════════════════════════════════════
//  SECTION 9 — FFT HELPER
// ═══════════════════════════════════════════════════════════════════════════════

void processFFT(float input[], float output[], float fft_return[2]) {
  fft_config_t *plan = fft_init(FFT_N, FFT_REAL, FFT_FORWARD, input, output);
  fft_execute(plan);

  float max_magnitude   = 0;
  float fundamental_freq = 0;
  for (int k = 1; k < plan->size / 2; k++) {
    float mag  = sqrt(pow(plan->output[2 * k], 2) + pow(plan->output[2 * k + 1], 2));
    float freq = (k * SAMPLE_RATE) / FFT_N;
    if (mag > max_magnitude) {
      max_magnitude    = mag;
      fundamental_freq = freq;
    }
  }
  fft_return[0] = fundamental_freq;
  fft_return[1] = max_magnitude;
  fft_destroy(plan);
}


// ═══════════════════════════════════════════════════════════════════════════════
//  SECTION 10 — HIT PROCESSING (PIXIE)
// ═══════════════════════════════════════════════════════════════════════════════

static TimerHandle_t hitProcessTimer = NULL;

static void hitProcessCallback(TimerHandle_t xTimer) {
  Serial.println("Pixie cooldown done.");
  digitalWrite(HIT_LED, LOW);
  for (pos = FINAL_POSITION; pos >= INITIAL_POSITION; pos -= 1) {
    myservo.write(pos);
    delay(MOVE_DELAY);
  }
  myDFPlayer.enableLoop();
  hit_state = WAITING;
}

int32_t hitProcessTimer_init(void) {
  hitProcessTimer = xTimerCreate(
    "hitTimer",
    pdMS_TO_TICKS(HIT_PROCESS_TIMER),
    pdFALSE, (void *)0,
    hitProcessCallback
  );
  if (hitProcessTimer == NULL) return -1;
  hit_state = WAITING;
  return 0;
}

void hitProcessTimer_start(void) {
  Serial.println("Pixie hit — starting cooldown timer.");
  if (hitProcessTimer != NULL) {
    xTimerReset(hitProcessTimer, 0);
    hit_state = PROCESSING;
  }
}

void process_hit() {
  myDFPlayer.advertise(1);
  digitalWrite(HIT_LED, HIGH);
  for (pos = INITIAL_POSITION; pos <= FINAL_POSITION; pos += 1) {
    myservo.write(pos);
    delay(MOVE_DELAY);
  }
  hitProcessTimer_start();
}


// ═══════════════════════════════════════════════════════════════════════════════
//  SECTION 11 — LEVIOSA TIMER
// ═══════════════════════════════════════════════════════════════════════════════

static TimerHandle_t leviosaProcessTimer = NULL;

static void leviosaProcessCallback(TimerHandle_t xTimer) {
  Serial.println("Leviosa cooldown done.");
  leviosa_state = WAITING;
}

int32_t leviosaProcessTimer_init(void) {
  leviosaProcessTimer = xTimerCreate(
    "leviosaTimer",
    pdMS_TO_TICKS(LEVIOSA_COOLDOWN_MS),
    pdFALSE, (void *)0,
    leviosaProcessCallback
  );
  if (leviosaProcessTimer == NULL) return -1;
  leviosa_state = WAITING;
  return 0;
}

void leviosaProcessTimer_start(void) {
  Serial.println("Leviosa — starting cooldown timer.");
  if (leviosaProcessTimer != NULL) {
    xTimerReset(leviosaProcessTimer, 0);
    leviosa_state = PROCESSING;
  }
}

// Spawn the FreeRTOS task that runs the animation sequence.
// Called from the FFT detection block — must not block.
void process_leviosa() {
  if (leviosaTaskHandle != NULL) return;  // already running (guard)
  leviosa_state = PROCESSING;
  currentPosition = 0;  // re-zero encoder on each cast
  xTaskCreate(
    leviosaSequenceTask,  // task function
    "LeviosaTask",        // name (debug)
    4096,                 // stack size (bytes) — increase if moveAndHold uses more
    NULL,                 // parameters
    1,                    // priority (1 = low, fine for motor control)
    &leviosaTaskHandle    // handle out
  );
}


// ═══════════════════════════════════════════════════════════════════════════════
//  SECTION 12 — LUMOS STUBS  (fill in per your LED implementation)
// ═══════════════════════════════════════════════════════════════════════════════

static TimerHandle_t lumosProcessTimer = NULL;

static void lumosProcessCallback(TimerHandle_t xTimer) {
  Serial.println("Lumos cooldown done.");
  // TODO: digitalWrite(LUMOS_LED_PIN, LOW);
  lumos_state = WAITING;
}

int32_t lumosProcessTimer_init(void) {
  lumosProcessTimer = xTimerCreate(
    "lumosTimer",
    pdMS_TO_TICKS(5000),  // TODO: pick your lumos duration
    pdFALSE, (void *)0,
    lumosProcessCallback
  );
  if (lumosProcessTimer == NULL) return -1;
  lumos_state = WAITING;
  return 0;
}

void lumosProcessTimer_start(void) {
  if (lumosProcessTimer != NULL) {
    xTimerReset(lumosProcessTimer, 0);
    lumos_state = PROCESSING;
  }
}

void process_lumos() {
  // TODO: digitalWrite(LUMOS_LED_PIN, HIGH);
  lumosProcessTimer_start();
}


// ═══════════════════════════════════════════════════════════════════════════════
//  SECTION 13 — SETUP
// ═══════════════════════════════════════════════════════════════════════════════

void setup() {
  // LEDs
  pinMode(LED_PIN,  OUTPUT); digitalWrite(LED_PIN,  HIGH);
  pinMode(HIT_LED,  OUTPUT); digitalWrite(HIT_LED,  HIGH);
  // TODO: pinMode(LEVIOSA_LED_PIN, OUTPUT); digitalWrite(LEVIOSA_LED_PIN, LOW);
  // TODO: pinMode(LUMOS_LED_PIN,   OUTPUT); digitalWrite(LUMOS_LED_PIN,   LOW);

  Serial.begin(115200);
  analogReadResolution(12);

  // Motor pins
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  ledcAttach(ENA_PIN, PWM_FREQ, PWM_RESOLUTION);
  motorStop();
  currentPosition = 0;

  // Encoder
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, RISING);

  // DFPlayer
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  delay(200);
  if (!myDFPlayer.begin(Serial2, false)) Serial.println("DFPlayer ERROR");
  delay(1000);
  myDFPlayer.volume(20);
  delay(500);
  myDFPlayer.loop(1);
  delay(500);
  myDFPlayer.enableLoop();

  // Timers
  hitProcessTimer_init();
  leviosaProcessTimer_init();
  lumosProcessTimer_init();

  // Servo
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  myservo.setPeriodHertz(50);
  myservo.attach(servoPin, 500, 2400);
  myservo.write(pos);

  digitalWrite(LED_PIN, LOW);
  digitalWrite(HIT_LED, LOW);

  Serial.println("Setup complete.");
}


// ═══════════════════════════════════════════════════════════════════════════════
//  SECTION 14 — MAIN LOOP
// ═══════════════════════════════════════════════════════════════════════════════

void loop() {
  unsigned long currentMicros    = micros();
  unsigned long currentMicrosPot = micros();

  // ── ADC sampling (both channels share the same sample rate) ────────────────
  if (currentMicros - previousMicros >= sampleInterval) {
    previousMicros = currentMicros;

    // Pixie channel
    fft_input[sampleIndex] = analogRead(ADC_PIN);

    // Leviosa channel — read on the same tick (negligible overhead)
    fft_input_leviosa[sampleIndex_leviosa] = analogRead(ADC_PIN_LEVIOSA);

    sampleIndex++;
    sampleIndex_leviosa++;

    // ── Process pixie FFT when buffer full ───────────────────────────────────
    if (sampleIndex >= FFT_N) {
      float fft_return[2];
      processFFT(fft_input, fft_output, fft_return);
      sampleIndex = 0;

      float fundamental_freq = fft_return[0];
      float max_magnitude    = fft_return[1];

      // --- Pixie hit detection ---
      if (fundamental_freq >= (TARGET_FREQ * 0.9) &&
          fundamental_freq <= (TARGET_FREQ * 1.1) &&
          max_magnitude    >= threshold &&
          hit_state        == WAITING) {
        Serial.println("Pixie hit detected!");
        process_hit();
      }

      // --- Lumos hit detection (same ADC_PIN; separate state) ---
      if (fundamental_freq >= (TARGET_FREQ * 0.9) &&
          fundamental_freq <= (TARGET_FREQ * 1.1) &&
          max_magnitude    >= threshold &&
          lumos_state      == WAITING) {
        Serial.println("Lumos hit detected!");
        process_lumos();
      }
    }

    // ── Process leviosa FFT when its buffer is full ──────────────────────────
    if (sampleIndex_leviosa >= FFT_N) {
      float fft_return_lev[2];
      processFFT(fft_input_leviosa, fft_output_leviosa, fft_return_lev);
      sampleIndex_leviosa = 0;

      float freq_lev = fft_return_lev[0];
      float mag_lev  = fft_return_lev[1];

      // --- Leviosa hit detection ---
      if (freq_lev   >= (TARGET_FREQ * 0.9) &&
          freq_lev   <= (TARGET_FREQ * 1.1) &&
          mag_lev    >= threshold &&
          leviosa_state == WAITING) {
        Serial.println("Leviosa hit detected!");
        process_leviosa();   // spawns FreeRTOS task — non-blocking
      }
    }
  }

  // ── Potentiometer polling (1 Hz) ────────────────────────────────────────────
  if (currentMicrosPot - previousMicrosPot >= checkPotInterval) {
    previousMicrosPot = currentMicrosPot;

    int volume_adc = analogRead(VOLUME_PIN);
    int volume_val = map(volume_adc, 0, MAX_ADC, 0, MAX_VOLUME);
    myDFPlayer.volume(volume_val);

    int state = myDFPlayer.readState();
    if (state == 2) {
      delay(500);
      digitalWrite(LED_PIN, HIGH);
      myDFPlayer.loop(1);
    }
  }
}