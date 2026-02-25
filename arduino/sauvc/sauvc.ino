#include <Servo.h>
#include "FlySkyIBus.h"

// === micro-ROS includes ===
#include <micro_ros_arduino.h>
#include <rmw_microros/rmw_microros.h>     // rmw_uros_ping_agent()
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <interfaces/msg/actuator.h>       // generated Actuator.msg header
#include <std_msgs/msg/bool.h>             // for heartbeat

// ================= Pins & Constants =================
const int LEFT_THRUSTER_PIN = 12;
const int RIGHT_THRUSTER_PIN = 13;
const int LED_PIN = LED_BUILTIN;
const int RELAY_PIN = 3;

const int NEUTRAL_SIGNAL = 1500;
const int MAX_FORWARD = 1600;
const int MAX_REVERSE = 1400;
const int MAX_FORWARD_AUTO = 1600;
const int MAX_REVERSE_AUTO = 1400;
const int SIGNAL_DEADBAND = 20;
const int IPR_DEADBAND = 20;

const int THROTTLE_CH = 1;
const int STEERING_CH = 0;
const int IPR_CH = 3;
const int MODE_CH = 4;
const int EMERGENCY_CH = 5;
const int LIGHT_CH = 8;

// ================= Globals =================
bool autoMode = false;

Servo leftThruster;
Servo rightThruster;

int lastTime = 0;
unsigned long myTime;

// Relay heartbeat state vars
bool relayState = false;
unsigned long lastRelayToggle = 0;
int heartbeatStage = 0;

bool relayState_rf = false;
unsigned long lastRelayToggle_rf = 0;
int heartbeatStage_rf = 0;

// ================= micro-ROS handles =================
rcl_node_t node;
rcl_subscription_t actuator_sub;
rcl_subscription_t heartbeat_sub;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;

interfaces__msg__Actuator last_actuator_msg;

// Static buffers for incoming actuator_cmd
double actuator_cmd_values_buffer[2];
rosidl_runtime_c__String actuator_cmd_names_buffer[2];
char cmd_name0[10];
char cmd_name1[10];
double actuator_cmd_covariance_buffer[2];  // if you expect 2 entries

bool actuator_msg_received = false;
std_msgs__msg__Bool last_heartbeat_msg;

// Globals to hold safe actuator values
double commanded_th_stbd = 0.0;
double commanded_th_port = 0.0;

// ================= micro-ROS publisher =================
rcl_publisher_t actuator_pub;
interfaces__msg__Actuator feedback_msg;

// Static buffers for feedback_msg
double actuator_values_buffer[2];
rosidl_runtime_c__String actuator_names_buffer[2];
char name0[10];
char name1[10];
double actuator_covariance_buffer[2];  // if you expect 2 entries

// ================= Fail-safe state =================
unsigned long last_actuator_msg_time = 0;
const unsigned long ACTUATOR_TIMEOUT_MS = 1000;  // 1 second watchdog

// ================= Connection state (auto-(re)connect) =================
bool mr_connected = false;
unsigned long last_ping_ms = 0;

// ================= Helper Functions =================
bool readSwitch(byte channelInput, bool defaultValue) {
  uint16_t ch = IBus.readChannel(channelInput);
  if (ch < 100) return defaultValue; // no signal

  const int ON_THRESHOLD = NEUTRAL_SIGNAL + 100;  // e.g. >1600 => ON
  const int OFF_THRESHOLD = NEUTRAL_SIGNAL - 100; // e.g. <1400 => OFF

  if (ch > ON_THRESHOLD) return true;
  if (ch < OFF_THRESHOLD) return false;
  // In the deadband around neutral, return the default or previous state.
  return defaultValue;
}

int readChannel(byte channelInput, int minLimit, int maxLimit, int defaultValue) {
  uint16_t ch = IBus.readChannel(channelInput);
  if (ch < 100) return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);
}

int applyDeadband(int value) {
  if (abs(value - NEUTRAL_SIGNAL) < SIGNAL_DEADBAND) {
    return NEUTRAL_SIGNAL;
  }
  return value;
}

// ================= micro-ROS Callbacks =================
void actuator_callback(const void * msgin) {
  const interfaces__msg__Actuator * msg = (const interfaces__msg__Actuator *)msgin;
  last_actuator_msg = *msg;
  actuator_msg_received = true;
  last_actuator_msg_time = millis();   // record reception time
  // SerialUSB.println("Actuator message received");
}

// ================= micro-ROS (create/destroy) =================
bool create_microros_entities() {
  rcl_ret_t rc;
  allocator = rcl_get_default_allocator();

  rc = rclc_support_init(&support, 0, NULL, &allocator);
  if (rc != RCL_RET_OK) {
    // SerialUSB.print("support init failed: "); SerialUSB.println(rc);
    return false;
  }

  rc = rclc_node_init_default(&node, "thruster_node", "", &support);
  if (rc != RCL_RET_OK) {
    // SerialUSB.print("node init failed: "); SerialUSB.println(rc);
    return false;
  }

  rc = rclc_subscription_init_default(
        &actuator_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(interfaces, msg, Actuator),
        "sookshma_01/actuator_cmd");
  if (rc != RCL_RET_OK) {
    // SerialUSB.print("actuator_sub init failed: "); SerialUSB.println(rc);
    return false;
  }

  rc = rclc_publisher_init_default(
        &actuator_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(interfaces, msg, Actuator),
        "sookshma_01/actuator_feedback");
  if (rc != RCL_RET_OK) {
    // SerialUSB.print("actuator_pub init failed: "); SerialUSB.println(rc);
    return false;
  }

  rc = rclc_executor_init(&executor, &support.context, 2, &allocator);
  if (rc != RCL_RET_OK) {
    // SerialUSB.print("executor init failed: "); SerialUSB.println(rc);
    return false;
  }

  // IMPORTANT: pass NULL so executor allocates the incoming message storage
  rc = rclc_executor_add_subscription(&executor, &actuator_sub, &last_actuator_msg, &actuator_callback, ON_NEW_DATA);
  if (rc != RCL_RET_OK) {
    // SerialUSB.print("add_sub actuator failed: "); SerialUSB.println(rc);
    return false;
  }

  mr_connected = true;
  digitalWrite(LED_PIN, HIGH);
  //SerialUSB.println("microros entities created OK");
  return true;
}

void destroy_microros_entities() {
  rcl_publisher_fini(&actuator_pub, &node);
  rcl_subscription_fini(&actuator_sub, &node);
  // rcl_subscription_fini(&heartbeat_sub, &node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
  mr_connected = false;
  digitalWrite(LED_PIN, LOW);    // disconnected indicator
}

// ================= Setup =================
void setup() {
  // Debug/aux UARTs (optional)
  // SerialUSB.begin(115200);       // your debug prints if needed
  Serial2.begin(115200);       // FlySky iBUS RX
  IBus.begin(Serial2);

  pinMode(RELAY_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);  // off until connected

  // Initialize thrusters
  leftThruster.attach(LEFT_THRUSTER_PIN);
  rightThruster.attach(RIGHT_THRUSTER_PIN);
  leftThruster.writeMicroseconds(NEUTRAL_SIGNAL);
  rightThruster.writeMicroseconds(NEUTRAL_SIGNAL);
  delay(2000);  // allow ESCs to init

  // === micro-ROS transport only (no entities in setup) ===
  Serial.begin(115200);                       // Programming Port baud
  set_microros_transports();     // Use Programming Port
  // If you switch to Native Port, use:
  // set_microros_serial_transports(SerialUSB);

  // ==== Static allocation of feedback_msg (safe even before connection) ====
  feedback_msg.actuator_values.data = actuator_values_buffer;
  feedback_msg.actuator_values.size = 2;
  feedback_msg.actuator_values.capacity = 2;

  feedback_msg.actuator_names.data = actuator_names_buffer;
  feedback_msg.actuator_names.size = 2;
  feedback_msg.actuator_names.capacity = 2;

  strcpy(name0, "th_stbd");
  feedback_msg.actuator_names.data[0].data = name0;
  feedback_msg.actuator_names.data[0].size = strlen(name0);
  feedback_msg.actuator_names.data[0].capacity = sizeof(name0);

  strcpy(name1, "th_port");
  feedback_msg.actuator_names.data[1].data = name1;
  feedback_msg.actuator_names.data[1].size = strlen(name1);
  feedback_msg.actuator_names.data[1].capacity = sizeof(name1);

  // Leave covariance empty
  feedback_msg.covariance.data = actuator_covariance_buffer;
  feedback_msg.covariance.size = 2;
  feedback_msg.covariance.capacity = 2;

  // ==== Static allocation of last_actuator_msg (safe even before connection) ====
  interfaces__msg__Actuator__init(&last_actuator_msg);

  // Allocate value array
  last_actuator_msg.actuator_values.data = actuator_cmd_values_buffer;
  last_actuator_msg.actuator_values.size = 2;
  last_actuator_msg.actuator_values.capacity = 2;

  // Allocate names array
  last_actuator_msg.actuator_names.data = actuator_cmd_names_buffer;
  last_actuator_msg.actuator_names.size = 2;
  last_actuator_msg.actuator_names.capacity = 2;

  // Assign string buffers (so incoming names have memory)
  last_actuator_msg.actuator_names.data[0].data = cmd_name0;
  last_actuator_msg.actuator_names.data[0].size = 0;
  last_actuator_msg.actuator_names.data[0].capacity = sizeof(cmd_name0);

  last_actuator_msg.actuator_names.data[1].data = cmd_name1;
  last_actuator_msg.actuator_names.data[1].size = 0;
  last_actuator_msg.actuator_names.data[1].capacity = sizeof(cmd_name1);

  // Allocate covariance if needed
  last_actuator_msg.covariance.data = actuator_cmd_covariance_buffer;
  last_actuator_msg.covariance.size = 2;
  last_actuator_msg.covariance.capacity = 2;
}

// ================= RF Manual Control =================
void Rf_control() {
  unsigned long currentTime = millis();
  if ((heartbeatStage_rf == 0 && currentTime - lastRelayToggle_rf >= 3000) ||
      (heartbeatStage_rf == 1 && currentTime - lastRelayToggle_rf >= 1000)) {
    relayState_rf = !relayState_rf;
    digitalWrite(RELAY_PIN, relayState_rf);
    lastRelayToggle_rf = currentTime;
    heartbeatStage_rf = (heartbeatStage_rf + 1) % 2;
  }

  int servoA_int = NEUTRAL_SIGNAL;
  int servoB_int = NEUTRAL_SIGNAL;

  // Normalized values [-1,1] to be published
  double th_stbd_norm = 0.0;
  double th_port_norm = 0.0;
  
  // Covariance values (small non-zero to indicate valid reading)
  double th_stbd_cov = 0.01;
  double th_port_cov = 0.01;

  int Ch2val = readChannel(THROTTLE_CH, MAX_REVERSE, MAX_FORWARD, NEUTRAL_SIGNAL);
  int Ch4val = readChannel(STEERING_CH, MAX_REVERSE, MAX_FORWARD, NEUTRAL_SIGNAL);

  // Apply deadband properly (overwrite value, not boolean-test)
  Ch2val = applyDeadband(Ch2val);
  Ch4val = applyDeadband(Ch4val);

  float steering = (Ch4val - NEUTRAL_SIGNAL) / (float)(MAX_FORWARD - NEUTRAL_SIGNAL);
  int   throttle_pwm = Ch2val;

  float turn_radius = 0.75f;
  int radius_val = (int)(turn_radius * (MAX_FORWARD_AUTO - NEUTRAL_SIGNAL));
  int steer_pwm  = (int)(steering * radius_val);

  servoA_int = constrain(throttle_pwm + steer_pwm, MAX_REVERSE_AUTO - radius_val, MAX_FORWARD_AUTO + radius_val);
  servoB_int = constrain(throttle_pwm - steer_pwm, MAX_REVERSE_AUTO - radius_val, MAX_FORWARD_AUTO + radius_val);

  leftThruster.writeMicroseconds(servoB_int);
  rightThruster.writeMicroseconds(servoA_int);

  // === Compute normalized values for feedback ===
  th_stbd_norm = (servoA_int - NEUTRAL_SIGNAL) / (double)(MAX_FORWARD_AUTO - NEUTRAL_SIGNAL);
  th_port_norm = (servoB_int - NEUTRAL_SIGNAL) / (double)(MAX_FORWARD_AUTO - NEUTRAL_SIGNAL);

  th_stbd_norm = constrain(th_stbd_norm, -1.0, 1.0);
  th_port_norm = constrain(th_port_norm, -1.0, 1.0);

  // === Publish RF feedback (only if connected) ===
  if (mr_connected) {
    feedback_msg.header.frame_id.data = "rf_control";
    feedback_msg.actuator_values.data[0] = th_stbd_norm;
    feedback_msg.actuator_values.data[1] = th_port_norm;
    feedback_msg.header.stamp.sec = millis() / 1000;
    feedback_msg.header.stamp.nanosec = (millis() % 1000) * 1000000;
    feedback_msg.covariance.data[0] = th_stbd_cov;
    feedback_msg.covariance.data[1] = th_port_cov;
    rcl_publish(&actuator_pub, &feedback_msg, NULL);

    // light spin so callbacks (e.g., heartbeat) still run in RF
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
  }
  // SerialUSB.println("RF Control Active");
}

// ================= AUTO Mode =================
void autoControl() {
  if (mr_connected) {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
  }

  // Normalized values [-1,1]
  static double th_stbd_norm = 0.0;
  static double th_port_norm = 0.0;

  // Covariance values (small non-zero to indicate valid reading)
  static double th_stbd_cov = 0.01;
  static double th_port_cov = 0.01;

  // Apply new actuator values when message arrives
  if (actuator_msg_received) {
    actuator_msg_received = false;
    
    th_stbd_norm = 0.0;
    th_port_norm = 0.0;
    th_stbd_cov = 0.01;
    th_port_cov = 0.01;

    for (size_t i = 0; i < last_actuator_msg.actuator_names.size; i++) {
      
      // SerialUSB.print("Actuator: ");
      // SerialUSB.print(last_actuator_msg.actuator_names.data[i].data);
      // SerialUSB.print(" Value: ");
      // SerialUSB.println(last_actuator_msg.actuator_values.data[i]);

      const char *name = last_actuator_msg.actuator_names.data[i].data;
      double value = last_actuator_msg.actuator_values.data[i];
      double cov = last_actuator_msg.covariance.data[i];

      if (strcmp(name, "th_stbd") == 0) {
        th_stbd_norm = value;
        th_stbd_cov = cov;
      } else if (strcmp(name, "th_port") == 0) {
        th_port_norm = value;
        th_port_cov = cov;
      }
    }
  }

  // === Fail-safe conditions ===
  bool timeout = (millis() - last_actuator_msg_time > ACTUATOR_TIMEOUT_MS);

  if (timeout) {
    // No recent command or heartbeat → neutral thrusters
    th_stbd_norm = 0.0;
    th_port_norm = 0.0;
    th_stbd_cov = 0.01;
    th_port_cov = 0.01;
    // SerialUSB.println("AUTO FAIL-SAFE: No Cmd ");    
  }

  // Map normalized [-1,1] to PWM
  int stbd_pwm = NEUTRAL_SIGNAL + (int)(th_stbd_norm * (MAX_FORWARD_AUTO - NEUTRAL_SIGNAL));
  int port_pwm = NEUTRAL_SIGNAL + (int)(th_port_norm * (MAX_FORWARD_AUTO - NEUTRAL_SIGNAL));

  stbd_pwm = constrain(stbd_pwm, MAX_REVERSE_AUTO, MAX_FORWARD_AUTO);
  port_pwm = constrain(port_pwm, MAX_REVERSE_AUTO, MAX_FORWARD_AUTO);

  rightThruster.writeMicroseconds(stbd_pwm);
  leftThruster.writeMicroseconds(port_pwm);

  // Always publish last known (or neutral) values (only if connected)
  if (mr_connected) {
    feedback_msg.header.frame_id.data = "auto_control";
    feedback_msg.actuator_values.data[0] = th_stbd_norm;
    feedback_msg.actuator_values.data[1] = th_port_norm;
    feedback_msg.header.stamp.sec = millis() / 1000;
    feedback_msg.header.stamp.nanosec = (millis() % 1000) * 1000000;
    feedback_msg.covariance.data[0] = th_stbd_cov;
    feedback_msg.covariance.data[1] = th_port_cov;
    rcl_publish(&actuator_pub, &feedback_msg, NULL);
  }
  // SerialUSB.println("Auto Control Active");
}

// ================= Loop =================
void loop() {
  // --- Non-blocking agent ping (~5 Hz) & connect/disconnect handling ---
  if (millis() - last_ping_ms > 200) {
    last_ping_ms = millis();
    bool agent_up = (rmw_uros_ping_agent(100, 1) == RMW_RET_OK);

    if (!mr_connected && agent_up) {
      create_microros_entities();    // bring up node/pubs/subs/executor
    } else if (mr_connected && !agent_up) {
      destroy_microros_entities();   // cleanly tear down; we'll re-create later
    }
  }

  // --- Rest of your logic ---
  myTime = millis() / 1000;
  if (myTime - lastTime > 1) {
    lastTime = myTime;
  }

  IBus.loop();
  int Ch5val = readSwitch(MODE_CH, false);

  if (Ch5val == 1) {
    autoMode = true;
    autoControl();
  } else {
    autoMode = false;
    Rf_control();
  }

  delay(10);
}