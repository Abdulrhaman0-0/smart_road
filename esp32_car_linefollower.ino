// =========================
// esp32_car_linefollower.ino
// =========================
// Subscribes to a single topic (e.g., "cars/N").
// Commands:
//   - "GO <ms>" -> enable motion with timeout
//   - "STOP"    -> immediate stop
// Drives L298N with a simple 3-IR line follower (fixed threshold).
//
// >>> IMPORTANT: SET YOUR UNIQUE MQTT CLIENT ID HERE (to avoid collisions):
// const char* MQTT_CLIENT_ID = "car_<YOUR_UNIQUE_ID>";  // e.g., car_N_21A3
//
// Also set CAR_TOPIC and Wi-Fi credentials below.

#include <WiFi.h>
#include <PubSubClient.h>

// ---------- WiFi / MQTT ----------
const char* WIFI_SSID = "YOUR_SSID";
const char* WIFI_PASS = "YOUR_PASS";

const char* MQTT_HOST = "broker.emqx.io";
const uint16_t MQTT_PORT = 1883;
const char* CAR_TOPIC  = "cars/N";             // change per car: cars/N or cars/S or cars/E or cars/W
const char* MQTT_CLIENT_ID = "car_SET_ME_UNIQUE";  // <<< WRITE A UNIQUE ID

WiFiClient net; 
PubSubClient mqtt(net);

// ---------- L298N pins ----------
const int ENA=25, IN1=26, IN2=27;   // left motor
const int ENB=14, IN3=12, IN4=13;   // right motor

// ---------- LEDC PWM ----------
const int PWM_CH_L = 0;
const int PWM_CH_R = 1;
const int PWM_BITS = 8;             // duty 0..255
const int PWM_FREQ = 20000;         // 20 kHz (quiet)

// ---------- IR pins & threshold ----------
const int IR_L=34, IR_C=35, IR_R=32;  // 34/35 input-only (ADC)
int IR_TH = 1800;                     // analog threshold (tune with your sensors)

// ---------- Motion ----------
int PWM_SPEED = 150;                // 0..255
bool allowed = false;
uint32_t goUntil = 0;

// ---------- Helpers ----------
void motorStop(){
  ledcWrite(PWM_CH_L, 0);
  ledcWrite(PWM_CH_R, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}
void motorForward(int lpwm, int rpwm){
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  ledcWrite(PWM_CH_L, lpwm);
  ledcWrite(PWM_CH_R, rpwm);
}
void motorLeft(){   motorForward((int)(PWM_SPEED*0.7), PWM_SPEED); }
void motorRight(){  motorForward(PWM_SPEED, (int)(PWM_SPEED*0.7)); }

int readAveraged(int pin, int samples=8){
  int sum = 0;
  for (int i=0;i<samples;i++) sum += analogRead(pin);
  return sum / samples;
}

// ---------- MQTT ----------
void onMsg(char* topic, byte* payload, unsigned int len){
  String s; s.reserve(len);
  for (unsigned int i=0;i<len;i++) s += (char)payload[i];
  s.trim();
  if (s.startsWith("GO")){
    int sp = s.indexOf(' ');
    uint32_t ms = (sp>0) ? (uint32_t) s.substring(sp+1).toInt() : 0;
    allowed = true;
    goUntil = millis() + (ms>0 ? ms : 5000);
    Serial.printf("[CMD] %s -> allowed for %u ms\n", s.c_str(), ms);
  } else if (s.startsWith("STOP")){
    allowed = false; motorStop();
    Serial.println("[CMD] STOP");
  } else {
    Serial.printf("[WARN] unknown payload: %s\n", s.c_str());
  }
}

void ensureMqtt(){
  while (!mqtt.connected()){
    Serial.print("MQTT...");
    if (mqtt.connect(MQTT_CLIENT_ID)) {   // <<< UNIQUE CLIENT ID
      Serial.println("connected");
      mqtt.subscribe(CAR_TOPIC, 1);
    } else {
      Serial.print("failed rc="); Serial.println(mqtt.state());
      delay(1000);
    }
  }
}

// ---------- Setup / Loop ----------
void setup(){
  Serial.begin(115200);

  // Motor pins
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  // LEDC PWM setup
  ledcSetup(PWM_CH_L, PWM_FREQ, PWM_BITS);
  ledcSetup(PWM_CH_R, PWM_FREQ, PWM_BITS);
  ledcAttachPin(ENA, PWM_CH_L); ledcWrite(PWM_CH_L, 0);
  ledcAttachPin(ENB, PWM_CH_R); ledcWrite(PWM_CH_R, 0);

  // IR inputs + ADC attenuation (better 0..3.3V range)
  pinMode(IR_L, INPUT); pinMode(IR_C, INPUT); pinMode(IR_R, INPUT);
  analogSetPinAttenuation(IR_L, ADC_11db);
  analogSetPinAttenuation(IR_C, ADC_11db);
  analogSetPinAttenuation(IR_R, ADC_11db);
  // analogReadResolution(12); // default 12-bit; uncomment if you change

  // Wi-Fi
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("WiFi");
  while (WiFi.status()!=WL_CONNECTED){ delay(300); Serial.print("."); }
  Serial.println(" OK");

  // MQTT
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(onMsg);
  ensureMqtt();
}

void loop(){
  if (!mqtt.connected()) ensureMqtt();
  mqtt.loop();

  // auto-stop when GO window ends
  if (allowed && (millis() > goUntil)) {
    allowed = false; motorStop();
    Serial.println("[TIMER] window ended -> STOP");
  }

  if (!allowed){
    motorStop();
    delay(5);
    return;
  }

  // --- 3-IR line-following ---
  int l = readAveraged(IR_L, 8);
  int c = readAveraged(IR_C, 8);
  int r = readAveraged(IR_R, 8);
  bool onL = (l < IR_TH), onC = (c < IR_TH), onR = (r < IR_TH);

  if (onC && !onL && !onR){
    motorForward(PWM_SPEED, PWM_SPEED);
  } else if (onL && !onR){
    motorLeft();
  } else if (onR && !onL){
    motorRight();
  } else {
    // ambiguous: go slower straight
    motorForward((int)(PWM_SPEED*0.6), (int)(PWM_SPEED*0.6));
  }

  delay(5);
}
