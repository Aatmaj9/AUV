#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/int32.h>

rcl_publisher_t publisher;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
std_msgs__msg__Int32 msg;

void setup() {

  Serial.begin(115200);
  delay(2000);   // VERY IMPORTANT on Portenta

  // Use WiFi transport instead of USB
  set_microros_wifi_transports(
    "mavlab",          // WiFi SSID
    "mavlab24",        // WiFi Password
    "192.168.0.43",    // micro-ROS Agent IP
    8888               // Agent port
  );

  allocator = rcl_get_default_allocator();

  rclc_support_init(&support, 0, NULL, &allocator);

  rclc_node_init_default(&node, "portenta_node", "", &support);

  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "portenta_topic"
  );

  msg.data = 0;
}

void loop() {
  msg.data++;
  rcl_publish(&publisher, &msg, NULL);
  delay(1000);
}