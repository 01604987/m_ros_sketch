const int trigPin = 32;
const int echoPin = 33;

#define SOUND_SPEED_DIV2 0.017 // Precomputed SOUND_SPEED / 2 for efficiency


enum TriggerState { IDLE, SET_HIGH, SET_LOW };
TriggerState triggerState = IDLE;

unsigned long triggerTimer = 0;
const unsigned long TRIG_HIGH_DURATION = 10; // 10 microseconds
volatile bool echo_received = false;
volatile unsigned long start_time = 0;
volatile unsigned long echo_duration = 0;

long duration;
float distanceCm;

unsigned long previousMillis = 0;
const unsigned long interval = 10; // Measure distance every 100 ms

void setup() {
  Serial.begin(115200);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  digitalWrite(trigPin, LOW); // Ensure trigger starts LOW
  attachInterrupt(digitalPinToInterrupt(echoPin), echoISR, CHANGE); // Attach interrupt
}

void echoISR() {
  if (digitalRead(echoPin) == HIGH) {
    start_time = micros(); // Rising edge: Start time
  } else {
    echo_duration = micros() - start_time; // Falling edge: Calculate duration
    echo_received = true; // Flag to process in loop
  }
}

void loop() {
  if (millis() - previousMillis >= interval) {
    
    unsigned long currentMicros = micros();

    // Non-blocking trigger logic
    if (triggerState == IDLE) {
      digitalWrite(trigPin, LOW);
      triggerTimer = currentMicros;
      triggerState = SET_HIGH;
    } else if (triggerState == SET_HIGH && (currentMicros - triggerTimer >= 2)) {
      digitalWrite(trigPin, HIGH);
      triggerTimer = currentMicros;
      triggerState = SET_LOW;
    } else if (triggerState == SET_LOW && (currentMicros - triggerTimer >= TRIG_HIGH_DURATION)) {
      digitalWrite(trigPin, LOW);
      triggerState = IDLE; // Reset to IDLE after completing the pulse

      previousMillis = millis();

    }

    if(echo_received) {
      echo_received = false;
      // Measure duration and calculate distance

      distanceCm = echo_duration * SOUND_SPEED_DIV2;

      Serial.print("Distance (cm): ");
      Serial.println(distanceCm);
    }
  }
  // Additional tasks can run here without being blocked by the trigger logic
}