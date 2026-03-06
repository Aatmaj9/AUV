/*
 * Micro-ROS Arduino Code for Portenta H7
 * * Features:
 * - Automatically reconnects if the micro-ROS Agent (Docker) is restarted or disconnected.
 * - Publishes an Int32 counter to the topic: "/portenta_counter" at 1Hz.
 * - Uses a non-blocking state machine to manage ROS 2 entity lifecycle (Create/Destroy).
 * - LED Status: ON (Connected), OFF (Searching for Agent).
 */

#include <micro_ros_arduino.h>
#include <rmw_microros/rmw_microros.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>

// === Handles ===
rcl_publisher_t publisher;
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_timer_t timer;
rclc_executor_t executor;
std_msgs__msg__Int32 msg;

// === State & Timing ===
bool device_connected = false;
unsigned long last_ping_ms = 0;

// Helper to check return codes
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  if (timer != NULL) {
    msg.data++;
    rcl_publish(&publisher, &msg, NULL);
  }
}

bool create_entities() {
  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "portenta_h7_node", "", &support));
  RCCHECK(rclc_publisher_init_default(
    &publisher, &node, 
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), 
    "portenta_counter"));

  RCCHECK(rclc_timer_init_default(
    &timer, &support, RCL_MS_TO_NS(1000), timer_callback));

  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  return true;
}

void destroy_entities() {
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&publisher, &node);
  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  set_microros_transports();
  msg.data = 0;
}

void loop() {
  // Check agent connection every 500ms
  if (millis() - last_ping_ms > 500) {
    last_ping_ms = millis();
    bool agent_alive = (rmw_uros_ping_agent(100, 1) == RMW_RET_OK);

    if (agent_alive && !device_connected) {
      if (create_entities()) {
        device_connected = true;
        digitalWrite(LED_BUILTIN, LOW); // Connected (Active Low)
      } else {
        destroy_entities();
      }
    } 
    else if (!agent_alive && device_connected) {
      destroy_entities();
      device_connected = false;
      digitalWrite(LED_BUILTIN, HIGH); // Disconnected
    }
  }

  if (device_connected) {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  }
}