#include "FFT.h"
#include "DFRobotDFPlayerMini.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include <ESP32Servo.h>

/////For Debugging/////
// TODO: Define LEVIOSA_LED_PIN (or LED strip pin) for the leviosa levitation effect
// TODO: Define LUMOS_LED_PIN (or LED strip pin) for the lumos light effect
///////////////////////

#define SAMPLE_RATE 16000       // Sample rate in Hz (50 kHz)
#define CHECK_RATE 1          // Sample rate in Hz (50 kHz)
#define FFT_N 1024              // FFT Size
#define ADC_PIN 15              // Change to your ADC pin
#define TARGET_FREQ 4000.0      // Frequency of wand signal
#define HIT_PROCESS_TIMER 5000  // Time before another hit can be detected after hit
#define LED_PIN 14            // Debugging LED
#define HIT_LED 12            // LEDs on bug guy
#define VOLUME_PIN 34         // Pin for volume potentiometer (Left pot)
#define THRESHOLD_PIN 35      // Pin for the threshold potentiometer (Right pot)
#define INITIAL_POSITION 85   // Starting position for the servo in degrees
#define FINAL_POSITION 175    // Hit position for the servo in degrees
#define MAX_ADC 4095          // Max adc value from the esp analog adc
#define MAX_VOLUME 30         // Max volume value
#define MAX_THRESHOLD 20000   // Max possible threshold 
#define MIN_THRESHOLD 100     // Min possible threshold

/////////////// Leviosa /////////////////////
#define ADC_PIN_LEVIOSA 36  // Second wand ADC for leviosa detection (adjust as needed)
#define ENA_PIN         25
#define IN1_PIN         26
#define IN2_PIN         27
#define ENCODER_A       32
#define ENCODER_B       33     // Can change any of these
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

// Leviosa FFT buffers (separate channel)
float fft_input_leviosa[FFT_N];
float fft_output_leviosa[FFT_N];
int   sampleIndex_leviosa = 0;
static TaskHandle_t leviosaTaskHandle = NULL;

// Motor helpers
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

//---------------------------PI Position Hold--------------------------------------------
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

//---------------------------- Smooth Glide for motor-------------------------
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

//----------------------- Leviosa FreeRTOS Task ------------------------------------
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

//------------------------------ Leviosa Timer ---------------------------------------
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
///////////////////////////////

#define MOVE_DELAY 10

DFRobotDFPlayerMini myDFPlayer; // Initialize the DFPlayer object
#define RXD2 5
#define TXD2 4

unsigned long previousMicros = 0;
unsigned long previousMicrosPot = 0;
const long sampleInterval = 1000000 / SAMPLE_RATE;  // Interval between samples (in microseconds)
const long checkPotInterval = 1000000 / CHECK_RATE;  // Interval between samples (in microseconds)

float fft_input[FFT_N];   // ADC sample buffer
float fft_output[FFT_N];  // FFT output buffer
int sampleIndex = 0;
int volatile checkCycle = 0;
float volatile threshold = 750;
typedef enum { WAITING,
               PROCESSING } hit_state_e;
hit_state_e hit_state = WAITING;
hit_state_e leviosa_state = WAITING;
hit_state_e lumos_state = WAITING;

Servo myservo;  // create servo object to control a servo
int pos = INITIAL_POSITION;    // Servo location
int servoPin = 13;

void setup() {
  // Initialize LEDs
  pinMode(LED_PIN, OUTPUT);   // Set digital pin as an output
  pinMode(HIT_LED, OUTPUT);   // Set digital pin as an output
  digitalWrite(LED_PIN, HIGH); // Turn the LED on
  digitalWrite(HIT_LED, HIGH); // Turn the LED on

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

  // TODO: pinMode LUMOS_LED_PIN as OUTPUT, initialize it LOW (off)

  Serial.begin(115200);
  analogReadResolution(12);  // 12-bit resolution (if available)

  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  delay(200);  // My preference for print stability

  if (!myDFPlayer.begin(Serial2, false)) {  // Start communication with DFPlayer, disable ack
    Serial.println("ERROR");
  }

  Serial.println();
  Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));

  delay(1000);  // Add this to allow player to fully initialise
  
  myDFPlayer.volume(20);  //Set volume value. From 0 to 30
  delay(500);
  myDFPlayer.loop(1);
  delay(500);
  myDFPlayer.enableLoop();

  hitProcessTimer_init();
  leviosaProcessTimer_init();
  // TODO: Initialize lumos timer (same pattern as hitProcessTimer_init)
  
  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  myservo.setPeriodHertz(50);           // standard 50 hz servo
  myservo.attach(servoPin, 500, 2400);  // attaches the servo on pin 18 to the servo object
                                        // using default min/max of 1000us and 2000us
                                        // different servos may require different min/max settings
                                        // for an accurate 0 to 180 sweep

  digitalWrite(LED_PIN, LOW); // Turn the LED on
  digitalWrite(HIT_LED, LOW); // Turn the LED on

  myservo.write(pos);  // Initialize to the standing position

  Serial.println("setup ended");  // I like this reassurance
}

void loop() {
  unsigned long currentMicros = micros();
  unsigned long currentMicrosPot = micros();

  // If it's time to take a sample
  if (currentMicros - previousMicros >= sampleInterval) {
    previousMicros = currentMicros;

    // Read ADC value and store it in the buffer
    fft_input[sampleIndex] = analogRead(ADC_PIN);  // Replace with your ADC pin
    
    // Leviosa channel — read on the same tick (negligible overhead)
    fft_input_leviosa[sampleIndex_leviosa] = analogRead(ADC_PIN_LEVIOSA);
    
    sampleIndex++;
    sampleIndex_leviosa++;

    // If the buffer is full, process the FFT
    if (sampleIndex >= FFT_N) {
      float fft_return[2];
      processFFT(fft_return);
      sampleIndex = 0;  // Reset the sample index for the next batch
      float fundamental_freq = fft_return[0];
      float max_magnitude = fft_return[1];
      // Serial.println(fundamental_freq);
      // Serial.println("Magnitude:\n");
      // Serial.println(max_magnitude);
      // Check for a hit then run the hit actions
      if (fundamental_freq >= (TARGET_FREQ * 0.9) && fundamental_freq <= (TARGET_FREQ * 1.1) && (max_magnitude >= threshold) && (hit_state == WAITING)) {  // Right freq, magnitude, and not too soon after last hit
        Serial.println("Hit detected!\n");
        process_hit();  // Process hit
      }

      
    }

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

    // TODO: Lumos hit detection - separate if block, NOT nested inside pixie hit above
      //       Read from fft_input_lumos[] buffer filled by ADC_PIN_LUMOS
      //       Same TARGET_FREQ and threshold check (same frequency, different pin)
      //       Check lumos_state == WAITING
      //       If all pass: call process_lumos()

  }

  // Check potentiometer values (100Hz Check)
  if (currentMicrosPot - previousMicrosPot >= checkPotInterval) {
    previousMicrosPot = currentMicrosPot;

    // if (checkCycle % 2 == 0) {  // Alternate between checks
    int volume_adc = analogRead(VOLUME_PIN);
    int volume_val = map(volume_adc, 0, MAX_ADC, 0, MAX_VOLUME);
    myDFPlayer.volume(volume_val);

    int state = myDFPlayer.readState(); // Get the playback state

    if (state == 2) { // 0 means the player is in the stopped state
      delay(500); // Wait to send another command
      digitalWrite(LED_PIN, HIGH); // Turn the LED on
      myDFPlayer.loop(1); // Replay or loop the desired file
    }
    // } else {
    //   int threshold_adc = analogRead(THRESHOLD_PIN);
    //   threshold = map(threshold_adc, 0, MAX_ADC, MIN_THRESHOLD, MAX_THRESHOLD);
    //   Serial.println(threshold_adc);
    //   Serial.println(threshold);
    // }
    // checkCycle++;

  }
}

// Function to process the FFT on the collected samples
void processFFT(float fft_return[2]) {
  // Execute the FFT (using the existing fft library you have)
  fft_config_t *real_fft_plan = fft_init(FFT_N, FFT_REAL, FFT_FORWARD, fft_input, fft_output);
  fft_execute(real_fft_plan);

  // Analyze the output (frequency bins)
  float max_magnitude = 0;
  float fundamental_freq = 0;
  for (int k = 1; k < real_fft_plan->size / 2; k++) {
    float mag = sqrt(pow(real_fft_plan->output[2 * k], 2) + pow(real_fft_plan->output[2 * k + 1], 2)) / 1;
    float freq = (k * SAMPLE_RATE) / FFT_N;

    // Track the max magnitude and fundamental frequency
    if (mag > max_magnitude) {
      max_magnitude = mag;
      fundamental_freq = freq;
    }
  }

  fft_return[0] = fundamental_freq;
  fft_return[1] = max_magnitude;

  // Clean up memory after FFT execution
  fft_destroy(real_fft_plan);
}

void process_hit() {
  /////////////////////////////Check for which target got hit////////////////////////////////
  
  // Play other song
  myDFPlayer.advertise(1);

  // Turn on hit
  digitalWrite(HIT_LED, HIGH); // Turn the LED on

  // Run servo
  for (pos = INITIAL_POSITION; pos <= FINAL_POSITION; pos += 1) {  // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);  // tell servo to go to position in variable 'pos'
    delay(MOVE_DELAY);           // waits 15ms for the servo to reach the position
  }

  // Timer that adds delay before another hit start
  hitProcessTimer_start();
}

static TimerHandle_t hitProcessTimer = NULL;  // Initialize a timer handle

// Timer callback (automatically resets xLockoutActive)
static void hitProcessCallback(TimerHandle_t xTimer) {
  Serial.println("done!");
  digitalWrite(HIT_LED, LOW); // Turn the LED on

  for (pos = FINAL_POSITION; pos >= INITIAL_POSITION; pos -= 1) {  // goes from 180 degrees to 0 degrees
    myservo.write(pos);                  // tell servo to go to position in variable 'pos'
    delay(MOVE_DELAY);                       // waits 15ms for the servo to reach the position
  }
  
  myDFPlayer.enableLoop();

  hit_state = WAITING;
}

// TODO: Add lumosProcessCallback - Ben has code to implement the motor functionality
// TODO: Add leviosaProcessCallback - Turn on for x amount of time, then turn off.

// Init the Timer
int32_t hitProcessTimer_init(void) {
  hitProcessTimer = xTimerCreate(
    "hitTimer",                        // Name
    pdMS_TO_TICKS(HIT_PROCESS_TIMER),  // Period (converted to ticks)
    pdFALSE,                           // Auto-reload = false (one-shot)
    (void *)0,                         // Timer ID (unused)
    hitProcessCallback                 // Callback function
  );

  if (hitProcessTimer == NULL) {
    return -1;  // Timer creation failed
  }
  hit_state = WAITING;
  return 0;
}

// TODO: Add lumosProcessTimer_init() here, parallel to hitProcessTimer_init()
//       - Same xTimerCreate pattern, pointing to lumosProcessCallback
//       - Set lumos_state = WAITING

// TODO: Add leviosaProcessTimer_init() here, parallel to hitProcessTimer_init()
//       - Same xTimerCreate pattern, pointing to leviosaProcessCallback
//       - Set leviosa_state = WAITING

// Start the timer from the beginning.
void hitProcessTimer_start(void) {
  // If initialized
  Serial.println("starting!");
  if (hitProcessTimer != NULL) {
    xTimerReset(hitProcessTimer, 0);  // Timer Restart
    hit_state = PROCESSING;
  }
}

// TODO: Add lumosProcessTimer_start() here, parallel to hitProcessTimer_start()
// TODO: Add leviosaProcessTimer_start() here, parallel to hitProcessTimer_start()
