#include <Servo.h>
#include "FlySkyIBus.h"

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
const int LIGHT_CH = 7;

bool autoMode = false;
bool serialConnected = false;

Servo leftThruster;
Servo rightThruster;


// Auto mode control values
float auto_throttle = 0.0;
float auto_steering = 0.0;
float auto_turn_radius = 0.5;

int lastTime = 0;
unsigned long myTime;

// Buffer for incoming serial data
#define BUFFER_SIZE 128  // Define the size of the buffer
char buffer[BUFFER_SIZE];  // Buffer to store incoming data
int bufferIndex = 0; 

// Add variables to track relay state and timing for heartbeat pattern
bool relayState = false;
unsigned long lastRelayToggle = 0;
int heartbeatStage = 0; // 0: ON for 1s, 1: OFF for 2s, 2: ON for 2s, 3: OFF for 2s

bool relayState_rf = false;
unsigned long lastRelayToggle_rf = 0;
int heartbeatStage_rf = 0; // 0: ON for 1s, 1: OFF for 2s, 2: ON for 2s, 3: OFF for 2s

bool readSwitch(byte channelInput, bool defaultValue) {
  int intDefaultValue = (defaultValue) ? 100 : 0;
  int ch = readChannel(channelInput, 0, 100, intDefaultValue);
  return (ch > 50);
}

int readChannel(byte channelInput, int minLimit, int maxLimit, int defaultValue) {
  uint16_t ch = IBus.readChannel(channelInput);
  if (ch < 100) return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);
}

int applyDeadband(float value) {
  if (abs(value - NEUTRAL_SIGNAL) < SIGNAL_DEADBAND) {
    return NEUTRAL_SIGNAL;
  }
  return value;
}

void setup() {
  Serial.begin(115200);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  // LED on indicates setup in progress

  Serial2.begin(115200);
  IBus.begin(Serial2);
  
  // Initialize thrusters first
  leftThruster.attach(LEFT_THRUSTER_PIN);
  rightThruster.attach(RIGHT_THRUSTER_PIN);
  
  // Set initial neutral position using writeMicroseconds
  leftThruster.writeMicroseconds(NEUTRAL_SIGNAL);
  rightThruster.writeMicroseconds(NEUTRAL_SIGNAL);
  
  delay(2000);  // Allow ESCs to initialize
  
  digitalWrite(LED_PIN, LOW);  // Setup complete
}



void Rf_control() {

  unsigned long currentTime = millis();
    if ((heartbeatStage_rf == 0 && currentTime - lastRelayToggle_rf >= 3000) ||  // ON for 1s
        (heartbeatStage_rf == 1 && currentTime - lastRelayToggle_rf >= 1000)  // OFF for 2s
        // (heartbeatStage == 2 && currentTime - lastRelayToggle >= 1000) ||  // ON for 2s
        // (heartbeatStage == 3 && currentTime - lastRelayToggle >= 3000) ||  // OFF for 2s (complete cycle)
        // (heartbeatStage == 4 && currentTime - lastRelayToggle >= 1000) ||  // ON for 2s
        // (heartbeatStage == 5 && currentTime - lastRelayToggle >= 3000)  // OFF for 2s (complete cycle)
        ) {  // OFF for 2s (complete cycle)
      
      // Toggle relay state
      relayState_rf = !relayState_rf;
      digitalWrite(RELAY_PIN, relayState_rf);
      
      // Update timing variables
      lastRelayToggle_rf = currentTime;
      heartbeatStage_rf = (heartbeatStage_rf + 1) % 2; // Cycle through 0,1,2,3
    }



  int servoA_int = 0;
  int servoB_int = 0;
  
  int Ch1val = readChannel(0, MAX_REVERSE, MAX_FORWARD, NEUTRAL_SIGNAL);
  int Ch2val = readChannel(1, MAX_REVERSE, MAX_FORWARD, NEUTRAL_SIGNAL);
  int Ch3val = readChannel(2, 0, MAX_FORWARD-NEUTRAL_SIGNAL, NEUTRAL_SIGNAL);
  int Ch4val = readChannel(3, MAX_REVERSE, MAX_FORWARD, NEUTRAL_SIGNAL);
  
  if (applyDeadband(Ch2val)) {
    float steering = (Ch4val-NEUTRAL_SIGNAL)/(float)(MAX_FORWARD-NEUTRAL_SIGNAL);
    float throttle = (Ch2val-NEUTRAL_SIGNAL)/(float)(MAX_FORWARD-NEUTRAL_SIGNAL);
    int throttle_pwm = Ch2val;
    
    float turn_radius = 0.5; 
    int radius_val = turn_radius * (MAX_FORWARD_AUTO - NEUTRAL_SIGNAL); // 50
    int steer_pwm = steering * radius_val; // -50 to 50 
    
    servoA_int = constrain(throttle_pwm + steer_pwm, MAX_REVERSE_AUTO-radius_val, MAX_FORWARD_AUTO+radius_val);
    servoB_int = constrain(throttle_pwm - steer_pwm, MAX_REVERSE_AUTO-radius_val, MAX_FORWARD_AUTO+radius_val);
    
    leftThruster.writeMicroseconds(servoB_int);
    rightThruster.writeMicroseconds(servoA_int);
  } else {
    leftThruster.writeMicroseconds(NEUTRAL_SIGNAL);
    rightThruster.writeMicroseconds(NEUTRAL_SIGNAL);
  }
}

void autoControl() {
        // Heartbeat pattern: ON 1s, OFF 2s, ON 2s, OFF 2s
    unsigned long currentTime = millis();
    if ((heartbeatStage == 0 && currentTime - lastRelayToggle >= 900) ||  // ON for 1s
        (heartbeatStage == 1 && currentTime - lastRelayToggle >= 200) ||  // OFF for 2s
        (heartbeatStage == 2 && currentTime - lastRelayToggle >= 300) ||  // ON for 2s
        (heartbeatStage == 3 && currentTime - lastRelayToggle >= 100) ||  // OFF for 2s (complete cycle)
        (heartbeatStage == 4 && currentTime - lastRelayToggle >= 300) ||  // ON for 2s
        (heartbeatStage == 5 && currentTime - lastRelayToggle >= 100)  // OFF for 2s (complete cycle)
        ) {  // OFF for 2s (complete cycle)
      
      // Toggle relay state
      relayState = !relayState;
      digitalWrite(RELAY_PIN, relayState);
      
      // Update timing variables
      lastRelayToggle = currentTime;
      heartbeatStage = (heartbeatStage + 1) % 6; // Cycle through 0,1,2,3
    }

    while (Serial.available()) {


      
    char incomingChar = Serial.read();  // Read one character at a time

    if (incomingChar == '\n') {  // Check for end of the message
      buffer[bufferIndex] = '\0';  // Null-terminate the string
      processMessage(buffer);     // Process the complete message
      bufferIndex = 0;            // Reset buffer index for next message
    } else {
      if (bufferIndex < BUFFER_SIZE - 1) {
        buffer[bufferIndex++] = incomingChar;  // Add to buffer
      } else {
        // Handle buffer overflow
        bufferIndex = 0;  // Reset buffer
        Serial.println("Buffer overflow, message ignored.");
      }
    }
  }
}


void processMessage(char* message) {
  // Convert the message into a String for easier processing
  String input = String(message);

  // Parse the message
  int commaIndex = input.indexOf(',');
  if (commaIndex != -1) {  // Ensure a valid format
    String steeringStr = input.substring(0, commaIndex);
    String throttleStr = input.substring(commaIndex + 1);

    // Convert strings to floats (expecting -1.0 to 1.0 range)
    float steering = steeringStr.toFloat();
    float throttle = throttleStr.toFloat();
    
    // Constrain inputs to valid ranges
    steering = constrain(steering, -1.0, 1.0);
    throttle = constrain(throttle, -1.0, 1.0);
    
    // Convert to PWM values
    float turn_radius = 0.5;
    int throttle_pwm = NEUTRAL_SIGNAL + (throttle * (MAX_FORWARD_AUTO - NEUTRAL_SIGNAL));
    int radius_val = turn_radius * (MAX_FORWARD_AUTO - NEUTRAL_SIGNAL); // 50
    int steer_pwm = steering * radius_val; // -50 to 50
    
    // Apply deadband and constrain outputs
    throttle_pwm = applyDeadband(throttle_pwm);
    int leftSignal = constrain(throttle_pwm + steer_pwm, MAX_REVERSE_AUTO-radius_val, MAX_FORWARD_AUTO+radius_val);
    int rightSignal = constrain(throttle_pwm - steer_pwm, MAX_REVERSE_AUTO-radius_val, MAX_FORWARD_AUTO+radius_val);
    
  //  leftThruster.writeMicroseconds(leftSignal);
  //   rightThruster.writeMicroseconds(rightSignal);

    leftThruster.writeMicroseconds(leftSignal);
    rightThruster.writeMicroseconds(rightSignal);
  } else {
    Serial.println("Invalid format received.");
  }
}

void loop() {
    myTime = millis()/1000;
    if (myTime - lastTime > 1) {
    //   Serial.println(myTime);
      lastTime = myTime;
    }
    
  
    
    IBus.loop();
    int Ch5val = readSwitch(MODE_CH, false);
    
  

  if (Ch5val == 1) {
    // Auto mode
    autoMode = true;
    autoControl();
  } else {
    // Manual RF control mode
    autoMode = false;
    Rf_control();
  }
  
  delay(10);  // Small delay for stability
}
