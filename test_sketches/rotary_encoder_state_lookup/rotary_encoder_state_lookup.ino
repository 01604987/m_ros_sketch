#define ENC_A 23  // ESP32 GPIO pin for encoder A
#define ENC_B 22  // ESP32 GPIO pin for encoder B

volatile int counter = 0;      // This variable will be changed by encoder input
volatile uint8_t old_AB = 0;      // Stores the previous state of the encoder pins

void setup() {
  pinMode(ENC_A, INPUT_PULLUP); // Enable internal pull-up resistor
  pinMode(ENC_B, INPUT_PULLUP); // Enable internal pull-up resistor

  Serial.begin(115200);
  Serial.println("Start");
}

void loop() {
  // Call the rotary encoder reading logic
  int8_t delta = read_encoder();
  if (delta != 0) {
    counter += delta;
    
    if (delta > 0) {
      Serial.print("Left ");
    } else{
      Serial.print("Right ");
    }
    Serial.print("Counter value: ");
    Serial.println(counter);
  }
}

/* Function to read the rotary encoder state */
int8_t read_encoder() {
  static const int8_t enc_states[] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
  uint8_t new_AB = 0;

  // Read current states of the encoder pins
  new_AB |= digitalRead(ENC_A) << 1; // Bit 1 is A
  new_AB |= digitalRead(ENC_B);      // Bit 0 is B

  // Compute the change in state based on the lookup table
  old_AB = ((old_AB << 2) | new_AB) & 0x0F; // Shift and mask the state
  return enc_states[old_AB];
}
