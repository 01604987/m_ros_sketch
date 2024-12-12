
#define A_PIN 25
#define B_PIN 26

volatile signed long temp, counter = 0; // This variable will increase or decrease depending on the rotation of encoder
volatile bool direction = true; // true = clockwise, false = counterclockwise
void setup() {
  Serial.begin(115200);

  pinMode(A_PIN, INPUT_PULLUP); // Internal pullup for GPIO 25
  pinMode(B_PIN, INPUT_PULLUP); // Internal pullup for GPIO 26

  // Setting up interrupts for GPIO 25 and GPIO 26 on ESP32
  attachInterrupt(digitalPinToInterrupt(A_PIN), ai0, RISING);
  attachInterrupt(digitalPinToInterrupt(B_PIN), ai1, RISING);
}

void loop() {
  // Send the value of counter
  if (counter != temp) {
    if (direction) {
      Serial.print("Clockwise ");
    } else {
      Serial.print("Counterclockwise ");
    }
    Serial.println(counter);

    temp = counter;
  }
}

void ai0() {
  // ai0 is activated if GPIO 25 is going from LOW to HIGH
  // Check pin 26 to determine the direction
  if (digitalRead(B_PIN) == LOW) {
    direction = true; // Clockwise
    counter++;
  } else {
    direction = false; // Clockwise
    counter--;
  }
}

void ai1() {
  // ai1 is activated if GPIO 26 is going from LOW to HIGH
  // Check pin 25 to determine the direction
  if (digitalRead(A_PIN) == LOW) {
    direction = false; // Clockwise

    counter--;
  } else {
    direction = true; // Clockwise

    counter++;
  }
}
