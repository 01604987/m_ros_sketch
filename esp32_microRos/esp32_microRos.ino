
/*
 * TODO : Include your necessary header here
*/

/////////////////////
/// For GPIO ////////
/////////////////////
#include <ezButton.h>

/////////////////////
/// For Micro ROS ///
/////////////////////
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
/*
 * TODO : include your desired msg header file
*/
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int16.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int16_multi_array.h>
#include <sensor_msgs/msg/imu.h>

/*
 * Optional,
 * LED pin to check connection
 * between micro-ros-agent and ESP32
*/


#define DEBOUNCE_TIME 50 // the debounce time in millisecond, increase this time if it still chatters
//define switch0
#define SWITCH0_PIN 18
ezButton toggleSwitch(SWITCH0_PIN);

//define rotary encoder pins

#define CLK_PIN 23 // GPIO34 lacking internal pull up/down. Only for input
#define DT_PIN  22 // GPIO35 lacking internal pull up/down. Only for input
#define SW_PIN  21 // ESP32 pin GPIO27 connected to the rotary encoder's button pin

//volatile int counter = 0;      // This variable will be changed by encoder input, instead use rotary_msg.data
volatile uint8_t prev_rot_encoder_state = 0;      // Stores the previous state of the encoder pins
ezButton button(SW_PIN); // create ezButton object that attach to pin GPIO21




// define optical rotary encoder pins

#define A_PIN 25 // GPIO25, input/output internal pull up/down
#define B_PIN 26 // GPIO26, input/output internal pull up/down
int ore_prev_position;


// define hcsr 04

#define TRIG 32
#define ECHO 33
#define SOUND_SPEED_DIV2 0.017 // Precomputed SOUND_SPEED / 2 for efficiency

enum TriggerState { IDLE, SET_HIGH, SET_LOW };
TriggerState trigger_state;
const uint8_t interval = 10; // interval for polling hcsr in ms
volatile unsigned long prev_mills = 0; // previous ms to keep non blocking loop
volatile unsigned long trigger_timer = 0; // temp time value to check trig low/high durations
const uint8_t TRIG_LOW_DURATION = 2; // 2 microseconds before setting trig to high
const uint8_t TRIG_HIGH_DURATION = 10; // 10 microseconds before setting trig back to low

volatile unsigned long echo_timer = 0; // temp timer to check for echo response. keeps loop non blockin
volatile bool echo_received = false; // requirement for publish call


volatile unsigned long duration; // calculated duration of echo
float prev_distance_cm; // previous distance to prevent successive data publish





// define linear potentiometer
// using ADC_2 because there seems to be some intereference with ADC_1 and echo/trigger readings
#define ADCPIN 4

int adc_value;
int prev_adc_value;


/*
 * Helper functions to help reconnect
*/
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
    static volatile int64_t init = -1; \
    if (init == -1) { init = uxr_millis();} \
    if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
  } while (0)

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

/*
 * Declare rcl object
*/
rclc_support_t support;
rcl_init_options_t init_options;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;
rcl_allocator_t allocator;

/*
 * TODO : Declare your 
 * publisher & subscription objects below
*/
rcl_publisher_t switch0_pub;
rcl_publisher_t rotary_encoder_pub;
rcl_publisher_t button_pub;
rcl_publisher_t ore_pub;
rcl_publisher_t hcsr04_pub;
rcl_publisher_t linear_pot_pub;
// rcl_publisher_t int16array_pub;
// rcl_subscription_t led_sub;
// rcl_subscription_t int16array_sub;
/*
 * TODO : Define your necessary Msg
 * that you want to work with below.
*/
std_msgs__msg__Bool switch0_msg;
std_msgs__msg__Int16 rotary_msg;
std_msgs__msg__Int16 button_msg;
std_msgs__msg__Int32 ore_msg;
std_msgs__msg__Float32 hcsr04_msg;
std_msgs__msg__Float32 linear_pot_msg;
// std_msgs__msg__Int16MultiArray int16array_send_msg;
// std_msgs__msg__Int16MultiArray int16array_recv_msg;
// std_msgs__msg__Bool led_msg;


/*
 * Define non ros callbacks
*/

void ch_A_handler(){
  // Triggered when Channel A goes from LOW to HIGH
  // Check Channel B to determine the rotation direction
  if (digitalRead(B_PIN) == LOW) {
    ore_msg.data++;
  } else {
    ore_msg.data--;
  }
}


void ch_B_handler(){
  // Triggered when Channel B goes from LOW to HIGH
  // Check Channel A to determine the rotation direction
  if (digitalRead(A_PIN) == LOW) {
    ore_msg.data--;
  } else {
    ore_msg.data++;
  }
}

void echo_ISR() {
  if (digitalRead(ECHO) == HIGH) {
    echo_timer = micros(); // Rising edge: Start time
  } else {
    duration = micros() - echo_timer; // Falling edge: Calculate duration
    echo_received = true; // Flag to process in loop
  }
}

/*
 * TODO : Define your subscription callbacks here
 * leave the last one as timer_callback()
*/

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  (void) last_call_time;
  if (timer != NULL) {

    /*
       TODO : Publish anything inside here
       
       For example, we are going to echo back
       the int16array_sub data to int16array_pub data,
       so we could see the data reflect each other.

       And also keep incrementing the button_pub
    */
    
    //rcl_publish(&button_pub, &button_msg, NULL);
  }
}

/*
   Create object (Initialization)
*/
bool create_entities()
{
  /*
     TODO : Define your
     - ROS node name
     - namespace
     - ROS_DOMAIN_ID
  */
  const char * node_name = "esp32";
  const char * ns = "";
  const int domain_id = 0;
  
  /*
   * Initialize node
   */
  allocator = rcl_get_default_allocator();
  init_options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&init_options, allocator);
  rcl_init_options_set_domain_id(&init_options, domain_id);
  rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
  rclc_node_init_default(&node, node_name, ns, &support);

  
  /*
   * TODO : Init your publisher and subscriber 
   */

  rclc_publisher_init(
    &switch0_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "/esp/switch0",
    &rmw_qos_profile_default
  );

  rclc_publisher_init(
    &rotary_encoder_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
    "/esp/rotary_encoder",
    &rmw_qos_profile_default
  );

  rclc_publisher_init(
    &button_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
    "/esp/rotary_button",
    &rmw_qos_profile_default
  );

  rclc_publisher_init(
    &ore_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/esp/optical_rotary_encoder",
    &rmw_qos_profile_default
  );

  rclc_publisher_init(
    &hcsr04_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/esp/hcsr04",
    &rmw_qos_profile_default
  );

  rclc_publisher_init(
    &linear_pot_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/esp/linear_pot",
    &rmw_qos_profile_default
  );
  
  // rclc_publisher_init(
  //   &int16array_pub,
  //   &node,
  //   ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
  //   "/esp/int16array_pub", &rmw_qos_profile_default);



  // rclc_subscription_init(
  //   &int16array_sub,
  //   &node,
  //   ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
  //   "/esp/int16array_sub", &rmw_qos_profile_default);

  // rclc_subscription_init(
  //   &led_sub,
  //   &node,
  //   ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
  //   "/esp/led", &rmw_qos_profile_default);

  /*
   * Init timer_callback
   * TODO : change timer_timeout
   * 50ms : 20Hz
   * 20ms : 50Hz
   * 10ms : 100Hz
   */
  const unsigned int timer_timeout = 50;
  rclc_timer_init_default(&timer,&support, RCL_MS_TO_NS(timer_timeout), timer_callback);

  /*
   * Init Executor
   * TODO : make sure the num_handles is correct
   * num_handles = total_of_subscriber + timer
   * publisher is not counted
   * 
   * TODO : make sure the name of sub msg and callback are correct
   */
  unsigned int num_handles = 1;
  executor = rclc_executor_get_zero_initialized_executor();
  rclc_executor_init(&executor, &support.context, num_handles, &allocator);
  //rclc_executor_add_subscription(&executor, &int16array_sub, &int16array_recv_msg, &int16array_callback, ON_NEW_DATA);
  //rclc_executor_add_subscription(&executor, &led_sub, &led_msg, &led_callback, ON_NEW_DATA);
  rclc_executor_add_timer(&executor, &timer);

  return true;
}
/*
 * Clean up all the created objects
 */
void destroy_entities()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_init_options_fini(&init_options);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
  /*
   * TODO : Make sue the name of publisher and subscriber are correct
   */
  
  rcl_publisher_fini(&switch0_pub, &node);
  rcl_publisher_fini(&rotary_encoder_pub, &node);
  rcl_publisher_fini(&ore_pub, &node);
  // rcl_publisher_fini(&int16array_pub, &node);
  rcl_publisher_fini(&button_pub, &node);
  rcl_publisher_fini(&hcsr04_pub, &node);
  rcl_publisher_fini(&linear_pot_pub, &node);
  // rcl_subscription_fini(&int16array_sub, &node);
  // rcl_subscription_fini(&led_sub, &node);
  
}

void setup() {


  /*
   * TODO : select either of USB or WiFi 
   * comment the one that not use
   */
  set_microros_transports();
  //set_microros_wifi_transports("WIFI-SSID", "WIFI-PW", "HOST_IP", 8888);

  /*
   * Optional, setup output pin for LEDs
   */
  // pinMode(LED_PIN, OUTPUT);
  // pinMode(LED_PIN_TEST, OUTPUT);

  // configure switch0
  toggleSwitch.setDebounceTime(DEBOUNCE_TIME);
  switch0_msg.data = false;

  // configure rotary encoder
  pinMode(CLK_PIN, INPUT_PULLUP);
  pinMode(DT_PIN, INPUT_PULLUP);
  button.setDebounceTime(DEBOUNCE_TIME);  // set debounce time to 50 milliseconds
  // initialize m_ros datatypes
  button_msg.data = 0;
  rotary_msg.data = 0;


  // configure optical rotary encoder
  pinMode(A_PIN, INPUT_PULLUP); // Channel A
  pinMode(B_PIN, INPUT_PULLUP); // Channel B
  // Setting up interrupts for encoder channels
  attachInterrupt(digitalPinToInterrupt(A_PIN), ch_A_handler, RISING);
  attachInterrupt(digitalPinToInterrupt(B_PIN), ch_B_handler, RISING);
  // initialize datatype
  ore_msg.data = 0;
  ore_prev_position = 0;

  // initialize hcsr 04 
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  // attach interrupt
  attachInterrupt(digitalPinToInterrupt(ECHO), echo_ISR, CHANGE); // Attach interrupt
  // trigger state
  trigger_state = IDLE;
  digitalWrite(TRIG, LOW); // Ensure trigger starts LOW


  /*
   * TODO : Initialze the message data variable
   */
  // int16_t array_data[2] = {0, 0};
  // int16array_send_msg.data.capacity = 2; // number of array length
  // int16array_send_msg.data.size = 2;     // number of array length
  // int16array_send_msg.data.data = array_data;

  // int16array_recv_msg.data.capacity = 2; // number of array length
  // int16array_recv_msg.data.size = 2;     // number of array length
  // int16array_recv_msg.data.data = array_data;



  // led_msg.data = false;

  /*
   * Setup first state
   */
  state = WAITING_AGENT;

}

/*
 * Define loop functions here
 */

/* Function to read the rotary encoder state */
int8_t read_encoder(uint8_t ENC_A, uint8_t ENC_B, volatile uint8_t* old_AB) {
  static const int8_t enc_states[] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
  uint8_t new_AB = 0;

  // Read current states of the encoder pins
  new_AB |= digitalRead(ENC_A) << 1; // Bit 1 is A
  new_AB |= digitalRead(ENC_B);      // Bit 0 is B

  // Compute the change in state based on the lookup table
  *old_AB = ((*old_AB << 2) | new_AB) & 0x0F; // Shift and mask the state
  return enc_states[*old_AB];
}

void loop() {
  /*
   * Try ping the micro-ros-agent (HOST PC), then switch the state 
   * from the example
   * https://github.com/micro-ROS/micro_ros_arduino/blob/galactic/examples/micro-ros_reconnection_example/micro-ros_reconnection_example.ino
   * 
   */
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      };
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
    default:
      break;
  }
  /*
   * Output LED when in AGENT_CONNECTED state
   */
  // if (state == AGENT_CONNECTED) {
  //   digitalWrite(LED_PIN, 1);

  // } else {
  //   digitalWrite(LED_PIN, 0);
  // }


  button.loop();
  toggleSwitch.loop();
  
  // rotary encoder
  // Call the rotary encoder reading logic
  int8_t delta = read_encoder(CLK_PIN, DT_PIN, &prev_rot_encoder_state);
  if (delta != 0) {
    rotary_msg.data += delta;
    rcl_publish(&rotary_encoder_pub, &rotary_msg, NULL);
  }


  // rotary encoder btn

  if (button.isPressed()){
    button_msg.data = 1;
    rcl_publish(&button_pub, &button_msg, NULL);
  }

  if (button.isReleased()){
    button_msg.data = 0;
    rcl_publish(&button_pub, &button_msg, NULL);
  }

  if (toggleSwitch.isPressed()){
    switch0_msg.data = false;
    rcl_publish(&switch0_pub, &switch0_msg, NULL);
  }

  if (toggleSwitch.isReleased()){
    switch0_msg.data = true;
    rcl_publish(&switch0_pub, &switch0_msg, NULL);
  }

  // optical rotary encoder

  // Check ore_pos
  if (ore_msg.data != ore_prev_position) {
    // publish position
    rcl_publish(&ore_pub, &ore_msg, NULL);
    ore_prev_position = ore_msg.data;
  }

  adc_value = analogRead(ADCPIN);
  if (adc_value != prev_adc_value){
    linear_pot_msg.data = adc_value / 4095.0;
    rcl_publish(&linear_pot_pub, &linear_pot_msg, NULL);
    prev_adc_value = adc_value;
  }


  // THIS CODE NEEDS OPTIMIZATION
  // OTHERWISE SLOWDOWN OF LOOP
  // Solution: subscribe to trigger in unity, only activate if hand close to speaker in game
  //
  if (millis() - prev_mills >= interval) {
    unsigned long current_micros = micros();

    // Non-blocking trigger logic
    if (trigger_state == IDLE) {
      digitalWrite(TRIG, LOW);
      trigger_timer = current_micros;
      trigger_state = SET_HIGH;
    } else if (trigger_state == SET_HIGH && (current_micros - trigger_timer >= TRIG_LOW_DURATION)) {
      digitalWrite(TRIG, HIGH);
      trigger_timer = current_micros;
      trigger_state = SET_LOW;
    } else if (trigger_state == SET_LOW && (current_micros - trigger_timer >= TRIG_HIGH_DURATION)) {
      digitalWrite(TRIG, LOW);
      trigger_state = IDLE; // Reset to IDLE after completing the pulse

      prev_mills = millis();
    }
  }

  if(echo_received) {
    echo_received = false;
    hcsr04_msg.data = duration * SOUND_SPEED_DIV2;

    if (hcsr04_msg.data != prev_distance_cm && hcsr04_msg.data <= 900) {
      rcl_publish(&hcsr04_pub, &hcsr04_msg, NULL);
      prev_distance_cm = hcsr04_msg.data; 
    }
  }




  /*
   * TODO : 
   * Do anything else you want to do here,
   * like read sensor data,  
   * calculate something, etc.
   */

}