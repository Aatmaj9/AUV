#include <micro_ros_arduino.h>

#include <rmw_microros/rmw_microros.h>

#include <rcl/rcl.h>

#include <rclc/rclc.h>

#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>



rcl_publisher_t publisher;

rcl_node_t node;

rclc_support_t support;

rcl_allocator_t allocator;

rcl_timer_t timer;

rclc_executor_t executor;

std_msgs__msg__Int32 msg;



// Diagnostic Blinker:

// Pattern 1 (Fast): Support failed

// Pattern 2 (Medium): Node failed

// Pattern 3 (Slow): Publisher failed

void error_loop(int blinks) {

while(1) {

for(int i=0; i<blinks; i++) {

digitalWrite(LED_BUILTIN, LOW); delay(150);

digitalWrite(LED_BUILTIN, HIGH); delay(150);

}

delay(1000); // Pause between patterns

}

}



void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {

if (timer != NULL) {

msg.data++;

rcl_publish(&publisher, &msg, NULL);

}

}



void setup() {

pinMode(LED_BUILTIN, OUTPUT);

digitalWrite(LED_BUILTIN, HIGH);

set_microros_transports();

allocator = rcl_get_default_allocator();



// Wait for Agent

while (RCL_RET_OK != rmw_uros_ping_agent(100, 1)) {

digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

delay(200);

}

digitalWrite(LED_BUILTIN, HIGH);



// --- DIAGNOSTIC STEP 1: SUPPORT ---

if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) error_loop(1);



// --- DIAGNOSTIC STEP 2: NODE ---

if (rclc_node_init_default(&node, "portenta_h7_node", "", &support) != RCL_RET_OK) error_loop(2);



// --- DIAGNOSTIC STEP 3: PUBLISHER ---

if (rclc_publisher_init_default(&publisher, &node,

ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "portenta_counter") != RCL_RET_OK) error_loop(3);



// --- DIAGNOSTIC STEP 4: TIMER & EXECUTOR ---

if (rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(1000), timer_callback) != RCL_RET_OK) error_loop(4);


executor = rclc_executor_get_zero_initialized_executor();

if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) error_loop(5);

if (rclc_executor_add_timer(&executor, &timer) != RCL_RET_OK) error_loop(6);



// If we get here, LED stays solid for 3 seconds then starts looping

digitalWrite(LED_BUILTIN, LOW); delay(3000); digitalWrite(LED_BUILTIN, HIGH);

}



void loop() {

if (RMW_RET_OK == rmw_uros_ping_agent(50, 1)) {

rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

}

}