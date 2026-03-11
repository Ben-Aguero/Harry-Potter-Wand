#include "FFT.h"
#include "DFRobotDFPlayerMini.h"
#include "freertos/FreeRTOS.h"
#include <ESP32Servo.h>

// TODO: Define LEVIOSA_LED_PIN (or LED strip pin) for the leviosa levitation effect
// TODO: Define LUMOS_LED_PIN (or LED strip pin) for the lumos light effect

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

  // TODO: pinMode LEVIOSA_LED_PIN as OUTPUT, intialize it LOW (off)
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

  // TODO: Initialize lumos timer (same pattern as hitProcessTimer_init)
  // TODO: Initialize leviosa timer (same pattern as hitProcessTimer_init)

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
    sampleIndex++;

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

      // TODO: Leviosa hit detection - separate if block, NOT nested inside pixie hit above
      //       Read from fft_input_leviosa[] buffer filled by ADC_PIN_LEVIOSA
      //       Same TARGET_FREQ and threshold check (same frequency, different pin)
      //       Check leviosa_state == WAITING
      //       If all pass: call process_leviosa()
      //       process_leviosa() must xTaskCreate a FreeRTOS task for the
      //       moveAndHold/glide sequence so it doesn't block FFT sampling

      // TODO: Lumos hit detection - separate if block, NOT nested inside pixie hit above
      //       Read from fft_input_lumos[] buffer filled by ADC_PIN_LUMOS
      //       Same TARGET_FREQ and threshold check (same frequency, different pin)
      //       Check lumos_state == WAITING
      //       If all pass: call process_lumos()
    }
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

// TODO: Add lumosProcessCallback
// TODO: Add leviosaProcessCallback

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
