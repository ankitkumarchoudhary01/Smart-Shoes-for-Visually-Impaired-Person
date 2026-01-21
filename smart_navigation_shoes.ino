/*
 * Smart Navigation Shoes - IoT Assistive Technology
 * For visually impaired individuals
 * 
 * Features:
 * - Multi-sensor obstacle detection (Ultrasonic + IR)
 * - Graduated haptic feedback
 * - Multi-modal alerts (vibration + audio + optional smartphone)
 * - Power management with auto-sleep
 * - Real-time processing with <100ms latency
 */

#include <Arduino.h>

// ============== PIN DEFINITIONS ==============
// Ultrasonic Sensors (HC-SR04)
#define ULTRASONIC_FRONT_TRIG    5
#define ULTRASONIC_FRONT_ECHO    18
#define ULTRASONIC_LEFT_TRIG     19
#define ULTRASONIC_LEFT_ECHO     21
#define ULTRASONIC_RIGHT_TRIG    22
#define ULTRASONIC_RIGHT_ECHO    23

// IR Proximity Sensors
#define IR_FRONT_PIN             34
#define IR_LEFT_PIN              35
#define IR_RIGHT_PIN             32

// Haptic Feedback (Vibration Motors)
#define VIBRATION_FRONT_PIN      25
#define VIBRATION_LEFT_PIN       26
#define VIBRATION_RIGHT_PIN      27

// Audio Alert
#define BUZZER_PIN               33

// Power Management
#define BATTERY_MONITOR_PIN      36
#define POWER_LED_PIN            2

// Optional:  Bluetooth for smartphone notifications
#define BT_ENABLED               true

// ============== CONFIGURATION ==============
#define MAX_DISTANCE             200  // Maximum detection range in cm
#define SAFE_DISTANCE            200  // Safe distance threshold (cm)
#define WARNING_DISTANCE         100  // Warning distance (cm)
#define DANGER_DISTANCE          50   // Danger distance (cm)
#define CRITICAL_DISTANCE        20   // Critical distance (cm)

#define VIBRATION_FREQUENCY      50   // Base vibration frequency (ms)
#define SENSOR_READ_INTERVAL     80   // Sensor reading interval (ms) for <100ms latency
#define BATTERY_CHECK_INTERVAL   60000 // Battery check every 60 seconds
#define LOW_BATTERY_THRESHOLD    3. 3  // Low battery voltage threshold
#define AUTO_SLEEP_TIMEOUT       300000 // Auto-sleep after 5 minutes of inactivity

// PWM Settings for vibration intensity
#define PWM_FREQ                 5000
#define PWM_RESOLUTION           8
#define PWM_CHANNEL_FRONT        0
#define PWM_CHANNEL_LEFT         1
#define PWM_CHANNEL_RIGHT        2

// ============== GLOBAL VARIABLES ==============
struct SensorData {
  float frontDistance;
  float leftDistance;
  float rightDistance;
  bool frontIRDetected;
  bool leftIRDetected;
  bool rightIRDetected;
  unsigned long lastDetectionTime;
};

SensorData sensors;
unsigned long lastSensorRead = 0;
unsigned long lastBatteryCheck = 0;
unsigned long lastActivityTime = 0;
bool isAsleep = false;
float batteryVoltage = 0;

// ============== SETUP ==============
void setup() {
  Serial.begin(115200);
  Serial.println("Smart Navigation Shoes - Initializing...");
  
  // Initialize pins
  initializePins();
  
  // Initialize PWM for vibration motors
  initializePWM();
  
  // Initialize Bluetooth (optional)
  #if BT_ENABLED
    initializeBluetooth();
  #endif
  
  // Startup feedback
  startupSequence();
  
  Serial.println("System Ready!");
  lastActivityTime = millis();
}

// ============== MAIN LOOP ==============
void loop() {
  unsigned long currentTime = millis();
  
  // Check for auto-sleep
  if (currentTime - lastActivityTime > AUTO_SLEEP_TIMEOUT && ! isAsleep) {
    enterSleepMode();
    return;
  }
  
  // Read sensors at defined intervals
  if (currentTime - lastSensorRead >= SENSOR_READ_INTERVAL) {
    lastSensorRead = currentTime;
    
    // Read all sensors
    readSensors();
    
    // Process sensor data and trigger alerts
    processObstacleDetection();
    
    // Update activity time if obstacle detected
    if (isObstacleDetected()) {
      lastActivityTime = currentTime;
      if (isAsleep) {
        wakeUp();
      }
    }
  }
  
  // Battery monitoring
  if (currentTime - lastBatteryCheck >= BATTERY_CHECK_INTERVAL) {
    lastBatteryCheck = currentTime;
    checkBattery();
  }
}

// ============== PIN INITIALIZATION ==============
void initializePins() {
  // Ultrasonic sensors
  pinMode(ULTRASONIC_FRONT_TRIG, OUTPUT);
  pinMode(ULTRASONIC_FRONT_ECHO, INPUT);
  pinMode(ULTRASONIC_LEFT_TRIG, OUTPUT);
  pinMode(ULTRASONIC_LEFT_ECHO, INPUT);
  pinMode(ULTRASONIC_RIGHT_TRIG, OUTPUT);
  pinMode(ULTRASONIC_RIGHT_ECHO, INPUT);
  
  // IR sensors
  pinMode(IR_FRONT_PIN, INPUT);
  pinMode(IR_LEFT_PIN, INPUT);
  pinMode(IR_RIGHT_PIN, INPUT);
  
  // Vibration motors
  pinMode(VIBRATION_FRONT_PIN, OUTPUT);
  pinMode(VIBRATION_LEFT_PIN, OUTPUT);
  pinMode(VIBRATION_RIGHT_PIN, OUTPUT);
  
  // Buzzer
  pinMode(BUZZER_PIN, OUTPUT);
  
  // Power management
  pinMode(BATTERY_MONITOR_PIN, INPUT);
  pinMode(POWER_LED_PIN, OUTPUT);
  digitalWrite(POWER_LED_PIN, HIGH);
}

// ============== PWM INITIALIZATION ==============
void initializePWM() {
  ledcSetup(PWM_CHANNEL_FRONT, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_LEFT, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_RIGHT, PWM_FREQ, PWM_RESOLUTION);
  
  ledcAttachPin(VIBRATION_FRONT_PIN, PWM_CHANNEL_FRONT);
  ledcAttachPin(VIBRATION_LEFT_PIN, PWM_CHANNEL_LEFT);
  ledcAttachPin(VIBRATION_RIGHT_PIN, PWM_CHANNEL_RIGHT);
}

// ============== SENSOR READING ==============
void readSensors() {
  // Read ultrasonic sensors
  sensors. frontDistance = readUltrasonic(ULTRASONIC_FRONT_TRIG, ULTRASONIC_FRONT_ECHO);
  sensors.leftDistance = readUltrasonic(ULTRASONIC_LEFT_TRIG, ULTRASONIC_LEFT_ECHO);
  sensors.rightDistance = readUltrasonic(ULTRASONIC_RIGHT_TRIG, ULTRASONIC_RIGHT_ECHO);
  
  // Read IR sensors (digital)
  sensors.frontIRDetected = digitalRead(IR_FRONT_PIN) == LOW;
  sensors.leftIRDetected = digitalRead(IR_LEFT_PIN) == LOW;
  sensors.rightIRDetected = digitalRead(IR_RIGHT_PIN) == LOW;
  
  // Debug output
  #ifdef DEBUG
  Serial.printf("Front: %. 2f cm, Left: %.2f cm, Right: %.2f cm\n", 
                sensors.frontDistance, sensors.leftDistance, sensors.rightDistance);
  Serial.printf("IR - Front: %d, Left: %d, Right:  %d\n", 
                sensors.frontIRDetected, sensors.leftIRDetected, sensors.rightIRDetected);
  #endif
}

// ============== ULTRASONIC SENSOR READING ==============
float readUltrasonic(int trigPin, int echoPin) {
  // Send trigger pulse
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Read echo pulse
  long duration = pulseIn(echoPin, HIGH, 30000); // 30ms timeout
  
  // Calculate distance in cm
  float distance = duration * 0.034 / 2;
  
  // Filter invalid readings
  if (distance == 0 || distance > MAX_DISTANCE) {
    return MAX_DISTANCE;
  }
  
  return distance;
}

// ============== OBSTACLE DETECTION PROCESSING ==============
void processObstacleDetection() {
  // Process front obstacle
  processDirectionalObstacle(sensors.frontDistance, sensors.frontIRDetected, 
                             PWM_CHANNEL_FRONT, "FRONT");
  
  // Process left obstacle
  processDirectionalObstacle(sensors.leftDistance, sensors.leftIRDetected, 
                             PWM_CHANNEL_LEFT, "LEFT");
  
  // Process right obstacle
  processDirectionalObstacle(sensors.rightDistance, sensors.rightIRDetected, 
                             PWM_CHANNEL_RIGHT, "RIGHT");
  
  // Trigger audio alert if critical obstacle detected
  if (sensors.frontDistance < CRITICAL_DISTANCE || 
      sensors.leftDistance < CRITICAL_DISTANCE || 
      sensors.rightDistance < CRITICAL_DISTANCE ||
      sensors.frontIRDetected || sensors.leftIRDetected || sensors.rightIRDetected) {
    triggerAudioAlert(true);
  } else if (sensors.frontDistance < DANGER_DISTANCE || 
             sensors.leftDistance < DANGER_DISTANCE || 
             sensors.rightDistance < DANGER_DISTANCE) {
    triggerAudioAlert(false);
  } else {
    digitalWrite(BUZZER_PIN, LOW);
  }
}

// ============== DIRECTIONAL OBSTACLE PROCESSING ==============
void processDirectionalObstacle(float distance, bool irDetected, int pwmChannel, const char* direction) {
  int vibrationIntensity = 0;
  
  // Sensor fusion:  Use IR for very close detection, ultrasonic for range
  if (irDetected) {
    // Critical:  Very close obstacle detected by IR
    vibrationIntensity = 255; // Maximum intensity
    Serial.printf("CRITICAL - %s: IR Detected!\n", direction);
    sensors.lastDetectionTime = millis();
  } else if (distance < CRITICAL_DISTANCE) {
    // Critical distance
    vibrationIntensity = 255;
    Serial.printf("CRITICAL - %s: %. 2f cm\n", direction, distance);
    sensors.lastDetectionTime = millis();
  } else if (distance < DANGER_DISTANCE) {
    // Danger distance - Strong vibration
    vibrationIntensity = map(distance, CRITICAL_DISTANCE, DANGER_DISTANCE, 255, 180);
    Serial.printf("DANGER - %s: %.2f cm\n", direction, distance);
    sensors.lastDetectionTime = millis();
  } else if (distance < WARNING_DISTANCE) {
    // Warning distance - Medium vibration
    vibrationIntensity = map(distance, DANGER_DISTANCE, WARNING_DISTANCE, 180, 100);
    Serial.printf("WARNING - %s: %.2f cm\n", direction, distance);
    sensors.lastDetectionTime = millis();
  } else if (distance < SAFE_DISTANCE) {
    // Far warning - Weak vibration
    vibrationIntensity = map(distance, WARNING_DISTANCE, SAFE_DISTANCE, 100, 50);
  } else {
    // Safe - No vibration
    vibrationIntensity = 0;
  }
  
  // Apply vibration intensity
  setVibrationIntensity(pwmChannel, vibrationIntensity);
  
  // Send smartphone notification if critical
  #if BT_ENABLED
  if (vibrationIntensity >= 200) {
    sendBluetoothAlert(direction, distance);
  }
  #endif
}

// ============== VIBRATION CONTROL ==============
void setVibrationIntensity(int pwmChannel, int intensity) {
  // Add pulsing effect for better perception
  static unsigned long lastPulseTime = 0;
  static bool pulseState = false;
  
  if (intensity > 0) {
    unsigned long currentTime = millis();
    int pulseInterval = map(intensity, 0, 255, 500, 100); // Faster pulse = closer obstacle
    
    if (currentTime - lastPulseTime >= pulseInterval) {
      lastPulseTime = currentTime;
      pulseState = !pulseState;
    }
    
    int actualIntensity = pulseState ?  intensity : intensity / 2;
    ledcWrite(pwmChannel, actualIntensity);
  } else {
    ledcWrite(pwmChannel, 0);
  }
}

// ============== AUDIO ALERT ==============
void triggerAudioAlert(bool critical) {
  static unsigned long lastBeepTime = 0;
  unsigned long currentTime = millis();
  
  if (critical) {
    // Continuous rapid beeping for critical obstacles
    if (currentTime - lastBeepTime >= 100) {
      digitalWrite(BUZZER_PIN, ! digitalRead(BUZZER_PIN));
      lastBeepTime = currentTime;
    }
  } else {
    // Slower beeping for warnings
    if (currentTime - lastBeepTime >= 300) {
      digitalWrite(BUZZER_PIN, ! digitalRead(BUZZER_PIN));
      lastBeepTime = currentTime;
    }
  }
}

// ============== BLUETOOTH INITIALIZATION ==============
#if BT_ENABLED
#include <BluetoothSerial.h>
BluetoothSerial SerialBT;

void initializeBluetooth() {
  SerialBT. begin("SmartNavShoes");
  Serial.println("Bluetooth initialized: SmartNavShoes");
}

void sendBluetoothAlert(const char* direction, float distance) {
  static unsigned long lastBTAlert = 0;
  unsigned long currentTime = millis();
  
  // Limit BT notifications to prevent spam
  if (currentTime - lastBTAlert >= 2000) {
    lastBTAlert = currentTime;
    char message[100];
    snprintf(message, sizeof(message), "ALERT|%s|%.1f\n", direction, distance);
    SerialBT.print(message);
  }
}
#endif

// ============== BATTERY MONITORING ==============
void checkBattery() {
  int adcValue = analogRead(BATTERY_MONITOR_PIN);
  batteryVoltage = (adcValue / 4095.0) * 3.3 * 2; // Assuming voltage divider
  
  Serial.printf("Battery Voltage: %.2fV\n", batteryVoltage);
  
  if (batteryVoltage < LOW_BATTERY_THRESHOLD) {
    // Low battery warning
    lowBatteryWarning();
  }
  
  #if BT_ENABLED
  char batteryMsg[50];
  snprintf(batteryMsg, sizeof(batteryMsg), "BATTERY|%.2f\n", batteryVoltage);
  SerialBT.print(batteryMsg);
  #endif
}

void lowBatteryWarning() {
  // Blink power LED
  for (int i = 0; i < 5; i++) {
    digitalWrite(POWER_LED_PIN, LOW);
    delay(200);
    digitalWrite(POWER_LED_PIN, HIGH);
    delay(200);
  }
  
  // Send alert
  Serial.println("WARNING: Low Battery!");
  #if BT_ENABLED
  SerialBT.println("BATTERY|LOW");
  #endif
}

// ============== POWER MANAGEMENT ==============
void enterSleepMode() {
  Serial.println("Entering sleep mode...");
  isAsleep = true;
  
  // Turn off all vibrations
  ledcWrite(PWM_CHANNEL_FRONT, 0);
  ledcWrite(PWM_CHANNEL_LEFT, 0);
  ledcWrite(PWM_CHANNEL_RIGHT, 0);
  digitalWrite(BUZZER_PIN, LOW);
  
  // Dim power LED
  digitalWrite(POWER_LED_PIN, LOW);
  
  #if BT_ENABLED
  SerialBT.println("STATUS|SLEEP");
  #endif
}

void wakeUp() {
  Serial.println("Waking up...");
  isAsleep = false;
  digitalWrite(POWER_LED_PIN, HIGH);
  
  #if BT_ENABLED
  SerialBT.println("STATUS|ACTIVE");
  #endif
}

// ============== UTILITY FUNCTIONS ==============
bool isObstacleDetected() {
  return (sensors.frontDistance < SAFE_DISTANCE || 
          sensors.leftDistance < SAFE_DISTANCE || 
          sensors.rightDistance < SAFE_DISTANCE ||
          sensors. frontIRDetected || 
          sensors.leftIRDetected || 
          sensors.rightIRDetected);
}

void startupSequence() {
  Serial.println("Running startup sequence...");
  
  // Test vibration motors
  ledcWrite(PWM_CHANNEL_FRONT, 200);
  delay(200);
  ledcWrite(PWM_CHANNEL_FRONT, 0);
  
  ledcWrite(PWM_CHANNEL_LEFT, 200);
  delay(200);
  ledcWrite(PWM_CHANNEL_LEFT, 0);
  
  ledcWrite(PWM_CHANNEL_RIGHT, 200);
  delay(200);
  ledcWrite(PWM_CHANNEL_RIGHT, 0);
  
  // Test buzzer
  digitalWrite(BUZZER_PIN, HIGH);
  delay(100);
  digitalWrite(BUZZER_PIN, LOW);
  
  #if BT_ENABLED
  SerialBT.println("STATUS|READY");
  #endif
}