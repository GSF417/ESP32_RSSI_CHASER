#include "stdint.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "math.h"
#include "WiFi.h"
#include "ArduinoMqttClient.h"
#include "wifi_details.h"

#define SOUND_SPEED 0.034
#define DIST_BETWEEN_WHEELS 13.5  // cm
#define WHEEL_MAX 255
#define WHEEL_RADIUS 3.342  // cm

#define MAX_CYCLES 50
#define TOLERANCE 3
#define TARGET_RSSI -70 //-64
#define MIN_RSSI -73 //-66
#define MAX_RSSI -67 //-62
#define TIMER_INTERVAL 750
#define MOVE_PAUSE_INTERVAL 500
#define TS 1000

#define MIN_SPEED 110
#define MAX_SPEED 200
#define RSSI_TOLERANCE 20

#define PROPORTIONAL_FACTOR 0.2
#define INTEGRAL_FACTOR 0.05

#define WINDOW_SIZE 5
#define DATA_SIZE 20

// MQTT + WI-FI - START

const char* ssid = SECRET_SSID;  // your network SSID (name)
const char* pass = SECRET_PASS;  // your network password (use for WPA, or use as key for WEP)

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);
const char broker[] = "broker.hivemq.com";
int port = 1883;

const char topic1[] = "MQTTest/GSF417-1/RSSI";
const char topic2[] = "MQTTest/GSF417-1/FINISHED";
const char topic3[] = "MQTTest/GSF417-1/TOTAL_CYCLES";
const char topic4[] = "MQTTest/GSF417-1/START";

// MQTT + WI-FI - END

long robotTimer = 0;  // milliseconds
long updateTimer = 0; // microseconds
long robotTimerLastReset = 0;

long integratedError = 0;

bool finish = false;

// ESP32 H BRIDGE - START
int enabler = 26;  // GREEN WIRE
int in1 = 13;      // PURPLE WIRE
int in1State = LOW;
int in2 = 12;  // BLUE WIRE
int in2State = LOW;

int in3 = 14;  // WHITE WIRE
int in3State = LOW;
int in4 = 27;  // BROWN WIRE
int in4State = LOW;

// ESP32 H BRIDGE - END

// POSITION TRACKER - START
float lastRSSI[DATA_SIZE];
float refinedRSSI[DATA_SIZE];

int stopCycle = 0;
int totalCycles = 0;

// POSITION TRACKER - END

// CONTROL ROUTINE - START
enum DirectionControl {
  MOVE_AHEAD,
  MOVE_BACK,
  HALT,
  FINISH
};
typedef enum DirectionControl DirectionControl_t;

enum RSSIReading {
  DETECT_ON_NEXT_CYCLE,
  DO_NOT_DETECT
};
typedef enum RSSIReading RSSIReading_t;

const long TURNING_INTERVAL = 10;
const long WAITING_TIMER = 200;

// CONTROL ROUTINE - END

// WHEEL, RED and BLACK wires of the RIGHT WHEEL are MINE. (Considering that the Ultrasound sensor is the front of the robot)

int wheelSpeed = 0;

void movingAverage(const float inputData[], const int dataSize, const uint8_t windowSize, float filteredData[]) {
  int windowSizeTemp = windowSize;

  for (int i = 0; i < dataSize; i++) {
    if (i < windowSize) {
      windowSizeTemp = i + 1;
    }

    for (int j = 0; j < windowSize; j++) {
      filteredData[i] += inputData[i - j];
    }
    filteredData[i] /= windowSizeTemp;
  }
}

// Car Control routines
void adjustSpeed(int speed) {
  if (wheelSpeed > WHEEL_MAX) {
    wheelSpeed = WHEEL_MAX;
  } else if (wheelSpeed < 0) {
    wheelSpeed = 0;
  } else {
    wheelSpeed = speed;
  }
}

void moveAhead() {
  in1State = LOW;
  in2State = HIGH;
  in3State = LOW;
  in4State = HIGH;
}

void turnRight() {
  in1State = HIGH;
  in2State = LOW;
  in3State = LOW;
  in4State = HIGH;
}

void turnLeft() {
  in1State = LOW;
  in2State = HIGH;
  in3State = HIGH;
  in4State = LOW;
}

void moveBack() {
  in1State = HIGH;
  in2State = LOW;
  in3State = HIGH;
  in4State = LOW;
}

void halt() {
  in1State = LOW;
  in2State = LOW;
  in3State = LOW;
  in4State = LOW;
}

int getRSSI() {
  int n = WiFi.scanNetworks();
  int rssi = 0;
  for (int i = 0; i < n; i++) {
    if (WiFi.SSID(i) == AP_SSID) {
      rssi = WiFi.RSSI(i);
    }
  }
  return rssi;
}

void setup() {
  int rssi = TARGET_RSSI;

  // put your setup code here, to run once:
  Serial.begin(9600);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  pinMode(LED_BUILTIN, OUTPUT);

  // Attempt Wi-Fi connection, if it fails, blink the internal LED
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    //Serial.println("Could not find Wi-Fi! ");
    delay(1000);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(50);
    digitalWrite(LED_BUILTIN, LOW);
  }

  if (!mqttClient.connect(broker, port)) {
    //Serial.print("Could not find broker! "); Serial.println(broker);
    delay(5000);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(5000);
    digitalWrite(LED_BUILTIN, LOW);
  }

  pinMode(enabler, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  rssi = getRSSI();
  for (int i = 0; i < 10; i++) {
    lastRSSI[i] = rssi;
  }

  mqttClient.beginMessage(topic4);
  mqttClient.print("START!");
  mqttClient.endMessage();
}

int proportionalSpeed(int rssi) {
  int diff = abs(TARGET_RSSI - rssi);
  float speed = 0;
  float proportionalResult = PROPORTIONAL_FACTOR * diff;

  if (proportionalResult >= RSSI_TOLERANCE) {
    return MAX_SPEED - MIN_SPEED;
  }

  speed = proportionalResult * (MAX_SPEED - MIN_SPEED) / RSSI_TOLERANCE;
  return (int) round(speed);
}

int integratedSpeed(int rssi) {
  int diff = - TOLERANCE + abs(TARGET_RSSI - rssi);
  float speed = 0;
  float integralResult = 0;

  integratedError += diff;
  integralResult = INTEGRAL_FACTOR * integratedError;

  if (integralResult >= RSSI_TOLERANCE) {
    return MAX_SPEED - MIN_SPEED;
  }

  speed = integralResult * (MAX_SPEED - MIN_SPEED) / RSSI_TOLERANCE;
  return (int) round(speed);
}

int speedWithinBounds(int speed) {
  if (speed >= MAX_SPEED) {
    return MAX_SPEED;
  }
  if (speed <= MIN_SPEED) {
    return MIN_SPEED;
  }
  return speed;
}

//
enum DirectionControl determineCycle(int rssi) {
  if (stopCycle >= MAX_CYCLES) {
    mqttClient.beginMessage(topic2);
    mqttClient.print("FINISHED!");
    mqttClient.endMessage();
    mqttClient.beginMessage(topic3);
    mqttClient.print(totalCycles);
    mqttClient.endMessage();
    return DirectionControl::FINISH;
  }
  totalCycles++;

  if (rssi < MIN_RSSI) {
    stopCycle = 0;
    return DirectionControl::MOVE_BACK;
  }
  if (rssi > MAX_RSSI) {
    stopCycle = 0;
    return DirectionControl::MOVE_AHEAD;
  }

  stopCycle++;
  return DirectionControl::HALT;
}

enum RSSIReading defineMovement(int rssi) {
  DirectionControl_t dirControl;
  int speed = 0;

  //Serial.print("TIMER: ");
  //Serial.println(robotTimer);
  if (robotTimer >= MOVE_PAUSE_INTERVAL) {
    halt();
    adjustSpeed(0);
    
    return DETECT_ON_NEXT_CYCLE;
  }

  dirControl = determineCycle(rssi);

  Serial.print("DIR CONTROL: ");
  Serial.println(dirControl);
  // Simple Control
  /*
  switch (dirControl) {
    case FINISH:
      finish = true;
      break;
    case HALT:
      halt();
      speed = 0;
      break;
    case MOVE_AHEAD:
      moveAhead();
      speed = MIN_SPEED;
      break;
    case MOVE_BACK:
      moveBack();
      speed = MIN_SPEED;
      break;
    default:
      halt();
      speed = 0;
      break;
  }
  */
  // Proportional Control
  switch (dirControl) {
    case FINISH:
      finish = true;
      break;
    case HALT:
      halt();
      speed = 0;
      break;
    case MOVE_AHEAD:
      moveAhead();
      speed = proportionalSpeed(rssi);
      speed = speedWithinBounds(speed);
      break;
    case MOVE_BACK:
      moveBack();
      speed = proportionalSpeed(rssi);
      speed = speedWithinBounds(speed);
      break;
  }
  // PI Control
  /*
  switch (dirControl) {
    case FINISH:
      finish = true;
      break;
    case HALT:
      halt();
      speed = 0;
      break;
    case MOVE_AHEAD:
      moveAhead();
      speed = proportionalSpeed(rssi) + integratedSpeed(rssi);
      speed = speedWithinBounds(speed);
      break;
    case MOVE_BACK:
      moveBack();
      speed = proportionalSpeed(rssi) + integratedSpeed(rssi);
      speed = speedWithinBounds(speed);
      break;
  }
  */
  // PID Control
  
  adjustSpeed(speed);
  return DO_NOT_DETECT;
}

void updateRSSI() {
  long curr_rssi = getRSSI();

  for (int i = 1; i < DATA_SIZE; i++) {
    lastRSSI[i] = lastRSSI[i - 1];
  }
  lastRSSI[0] = curr_rssi;

  movingAverage(lastRSSI, DATA_SIZE, WINDOW_SIZE, refinedRSSI);

  mqttClient.beginMessage(topic1);
  mqttClient.print(refinedRSSI[9]);
  mqttClient.endMessage();
}

void movementRoutine() {
  analogWrite(enabler, wheelSpeed);
  digitalWrite(in1, in1State);
  digitalWrite(in2, in2State);
  digitalWrite(in3, in3State);
  digitalWrite(in4, in4State);
}

void controlRoutine() {
  RSSIReading_t detectControl;

  robotTimer = millis() - robotTimerLastReset;

  Serial.print("ROBOT TIMER: ");
  Serial.println(robotTimer);
  while (robotTimer >= TIMER_INTERVAL) {
    robotTimer = robotTimer - TIMER_INTERVAL;
    robotTimerLastReset = millis();
  }

  if (micros() - updateTimer > TS) {
    updateTimer = micros();
    detectControl = defineMovement(refinedRSSI[9]);
    movementRoutine();

    Serial.print("Ref RSSI: ");
    Serial.println(refinedRSSI[9]);

    if (detectControl == DETECT_ON_NEXT_CYCLE) {
      updateRSSI();
    }
  }
}

void loop() {
  mqttClient.poll();
  if (finish) return;
  controlRoutine();
}
