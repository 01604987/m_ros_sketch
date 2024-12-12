#include "driver/timer.h"

const int trigPin = 32;
const int echoPin = 33;

#define SOUND_SPEED_DIV2 0.017 // Precomputed SOUND_SPEED / 2 for efficiency

volatile bool readyToMeasure = false; // Flag to start a new cycle
volatile bool start_timer_2 = false;
volatile bool start_timer_3 = false;
volatile long duration = 0;
volatile float distanceCm = 0;

void IRAM_ATTR onTimer1(void* arg); // Poll every 100 ms
void IRAM_ATTR onTimer2(void* arg); // 2 µs delay
void IRAM_ATTR onTimer3(void* arg); // 10 µs HIGH pulse

void setup() {
  Serial.begin(115200);

  // Initialize pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  digitalWrite(trigPin, LOW);

  // Configure Timer 1 (100 ms polling)
  setupTimer(TIMER_GROUP_0, TIMER_0, 100000, &onTimer1, true); // 100 ms
  setupTimer(TIMER_GROUP_0, TIMER_1, 2, &onTimer2, false);     // 2 µs delay
  setupTimer(TIMER_GROUP_1, TIMER_0, 10, &onTimer3, false);    // 10 µs delay (corrected)
}

void loop() {
  if (start_timer_2){
    timer_set_counter_value(TIMER_GROUP_0, TIMER_1, 0);
    timer_start(TIMER_GROUP_0, TIMER_1);
    start_timer_2 = false; // Ensure the flag is reset
  }
  
  if (start_timer_3) {
    timer_set_counter_value(TIMER_GROUP_1, TIMER_0, 0);
    timer_start(TIMER_GROUP_1, TIMER_0);
    start_timer_3 = false; // Ensure the flag is reset (added)
  }

  if (readyToMeasure) {
    // Measure the echo duration
    duration = pulseIn(echoPin, HIGH);
    distanceCm = duration * SOUND_SPEED_DIV2;
    // Print the distance
    Serial.print("Distance (cm): ");
    Serial.println(distanceCm);
    // Reset the flag
    readyToMeasure = false;
  }

  delay(10); // Small delay to prevent overwhelming the loop
}

// Timer setup function
void setupTimer(timer_group_t group, timer_idx_t timer, uint64_t interval_us, void (*callback)(void*), bool auto_reload) {
  timer_config_t config = {
      .alarm_en = TIMER_ALARM_EN,
      .counter_en = TIMER_PAUSE,
      .intr_type = TIMER_INTR_LEVEL,
      .counter_dir = TIMER_COUNT_UP,
      .auto_reload = auto_reload ? TIMER_AUTORELOAD_EN : TIMER_AUTORELOAD_DIS,
      .divider = 80 // 1 microsecond per tick (80 MHz / 80 = 1 MHz)
  };
  timer_init(group, timer, &config);
  timer_set_counter_value(group, timer, 0);
  timer_set_alarm_value(group, timer, interval_us);
  timer_enable_intr(group, timer);
  timer_isr_register(group, timer, callback, NULL, ESP_INTR_FLAG_IRAM, NULL);

  // Start cyclic timers immediately, one-shot timers will be started on demand
  if (auto_reload) {
    timer_start(group, timer);
  }
}

// Callback for the 1st timer (100 ms polling interval)
void IRAM_ATTR onTimer1(void* arg) {
  digitalWrite(trigPin, LOW); // Ensure the trigger pin is LOW
  start_timer_2 = true;       // Signal to start Timer 2 in loop()
}

// Callback for Timer 2 (2 µs delay)
void IRAM_ATTR onTimer2(void* arg) {
  digitalWrite(trigPin, HIGH); // Start the HIGH pulse
  start_timer_3 = true;        // Signal to start Timer 3 in loop()
}

// Callback for Timer 3 (10 µs HIGH pulse)
void IRAM_ATTR onTimer3(void* arg) {
  digitalWrite(trigPin, LOW);  // End the HIGH pulse
  readyToMeasure = true;       // Signal that we're ready to measure
}