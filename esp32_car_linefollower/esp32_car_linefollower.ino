#include <WiFi.h>
#include <PubSubClient.h>
#include <cstring>
#include <cstdlib>

// --------------- User configuration ----------------
const char* WIFI_SSID = "YOUR_SSID";
const char* WIFI_PASS = "YOUR_PASSWORD";

const char* MQTT_HOST = "broker.emqx.io";
const uint16_t MQTT_PORT = 1883;
const char* MQTT_CLIENT_ID = "esp32_car";
const char* CAR_TOPIC = "cars/N";  // change per car (cars/N, cars/S, cars/E, cars/W)

// Motor pins (L298N)
const int PIN_ENA = 25;
const int PIN_IN1 = 26;
const int PIN_IN2 = 27;
const int PIN_ENB = 14;
const int PIN_IN3 = 12;
const int PIN_IN4 = 13;

// Line sensors (analog)
const int PIN_IR_LEFT   = 34;
const int PIN_IR_CENTER = 35;
const int PIN_IR_RIGHT  = 32;

// PWM configuration
const int PWM_FREQ = 2000;
const int PWM_RES  = 8;      // bits
const int BASE_SPEED = 200;  // 0-255
const int TURN_SPEED = 150;  // slower speed for correcting
const int IR_THRESHOLD = 1800;  // adjust depending on tape/lighting

// ----------------------------------------------------

WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

bool runActive = false;
unsigned long runEndsAt = 0;

void ensureWifi() {
  if (WiFi.status() == WL_CONNECTED) {
    return;
  }
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" connected");
}

void ensureMqtt() {
  while (!mqtt.connected()) {
    Serial.print("Connecting to MQTT...");
    if (mqtt.connect(MQTT_CLIENT_ID)) {
      Serial.println("connected");
      mqtt.subscribe(CAR_TOPIC);
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqtt.state());
      Serial.println(" retrying in 2s");
      delay(2000);
    }
  }
}

void setMotors(int leftSpeed, int rightSpeed, int leftDir, int rightDir) {
  leftSpeed  = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  // leftDir/rightDir: 0 = stop, 1 = forward, 2 = backward
  digitalWrite(PIN_IN1, (leftDir == 1) ? HIGH : LOW);
  digitalWrite(PIN_IN2, (leftDir == 2) ? HIGH : LOW);
  digitalWrite(PIN_IN3, (rightDir == 1) ? HIGH : LOW);
  digitalWrite(PIN_IN4, (rightDir == 2) ? HIGH : LOW);

  // With ESP32 core 3.x: use pin directly in ledcWrite
  ledcWrite(PIN_ENA, leftSpeed);
  ledcWrite(PIN_ENB, rightSpeed);
}

void stopMotors() {
  ledcWrite(PIN_ENA, 0);
  ledcWrite(PIN_ENB, 0);

  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, LOW);
}

bool readSensor(int pin) {
  int value = analogRead(pin);
  // line assumed darker (lower reading)
  return value < IR_THRESHOLD;
}

void followLine() {
  bool left   = readSensor(PIN_IR_LEFT);
  bool center = readSensor(PIN_IR_CENTER);
  bool right  = readSensor(PIN_IR_RIGHT);

  if (center) {
    // go straight
    setMotors(BASE_SPEED, BASE_SPEED, 1, 1);
  } else if (left && !right) {
    // turn left
    setMotors(TURN_SPEED, BASE_SPEED, 1, 1);
  } else if (right && !left) {
    // turn right
    setMotors(BASE_SPEED, TURN_SPEED, 1, 1);
  } else if (left && right) {
    // both see line: slow straight (junction or thick line)
    setMotors(TURN_SPEED, TURN_SPEED, 1, 1);
  } else {
    // lost line: stop
    stopMotors();
  }
}

void handleCommand(const char* payload) {
  if (strncmp(payload, "GO", 2) == 0) {
    unsigned long duration = strtoul(payload + 2, nullptr, 10);
    if (duration == 0) {
      Serial.println("GO command missing duration");
      return;
    }
    runActive = true;
    runEndsAt = millis() + duration;
    Serial.printf("GO for %lu ms\n", duration);
  } else if (strncmp(payload, "STOP", 4) == 0) {
    runActive = false;
    stopMotors();
    Serial.println("STOP received");
  } else {
    Serial.print("Unknown payload: ");
    Serial.println(payload);
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  if (strcmp(topic, CAR_TOPIC) != 0) {
    return;
  }

  static char buffer[48];
  size_t copyLen = length;
  if (copyLen >= sizeof(buffer)) {
    copyLen = sizeof(buffer) - 1;
  }

  memcpy(buffer, payload, copyLen);
  buffer[copyLen] = '\0';

  handleCommand(buffer);
}

void updateRun() {
  unsigned long now = millis();
  if (runActive && now >= runEndsAt) {
    runActive = false;
    stopMotors();
    Serial.println("Run window elapsed -> STOP");
  }

  if (runActive) {
    followLine();
  } else {
    stopMotors();
  }
}

void setup() {
  Serial.begin(115200);

  // Motor control pins
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);

  // Line sensor pins
  pinMode(PIN_IR_LEFT, INPUT);
  pinMode(PIN_IR_CENTER, INPUT);
  pinMode(PIN_IR_RIGHT, INPUT);

  // PWM attach (new API in ESP32 core 3.x)
  ledcAttach(PIN_ENA, PWM_FREQ, PWM_RES); // pin, freq, resolution bits
  ledcAttach(PIN_ENB, PWM_FREQ, PWM_RES);

  ensureWifi();
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
  ensureMqtt();
}

void loop() {
  ensureWifi();
  ensureMqtt();
  mqtt.loop();
  updateRun();
  delay(5);
}
