// ==========================
// esp32_signals_controller.ino
// ==========================
// Subscribes to "signals/cycle" (QoS1).
// Payload format (single line):
//   "CYCLE <ORDER> <NS_ms> <EW_ms> <AMBER_ms> <ALLRED_ms>"
// ORDER: "NS" or "EW" (who goes first).
//
// >>> IMPORTANT: SET YOUR UNIQUE MQTT CLIENT ID HERE (to avoid collisions):
// const char* MQTT_CLIENT_ID = "signals_<YOUR_UNIQUE_ID>";  // e.g., signals_21A3
//
// Also set your Wi-Fi credentials below.

#include <WiFi.h>
#include <PubSubClient.h>
#include <stdio.h>  // for sscanf

// ---------- WiFi / MQTT ----------
const char* WIFI_SSID = "YOUR_SSID";
const char* WIFI_PASS = "YOUR_PASS";

const char* MQTT_HOST = "broker.emqx.io";
const uint16_t MQTT_PORT = 1883;
const char* SUB_TOPIC = "signals/cycle";

const char* MQTT_CLIENT_ID = "signals_SET_ME_UNIQUE"; // <<< WRITE A UNIQUE ID

WiFiClient espClient;
PubSubClient mqtt(espClient);

// ---------- Pin mapping (12 LEDs) ----------
const int N_R=21, N_Y=22, N_G=23;
const int S_R=19, S_Y=18, S_G=5;
const int E_R=25, E_Y=26, E_G=27;
const int W_R=32, W_Y=33, W_G=4;

// ---------- Cycle state ----------
struct Cycle {
  bool nsFirst = true;
  uint32_t nsG = 8000, ewG = 8000, amber = 2000, allred = 1000;
};
Cycle active, pending;
volatile bool hasPending = false;

enum Phase { PH_NS_G, PH_NS_Y, PH_ALLRED1, PH_EW_G, PH_EW_Y, PH_ALLRED2 };
Phase phase = PH_NS_G;
uint32_t tStart = 0; // ms of current phase

void setNS(bool r,bool y,bool g) { digitalWrite(N_R,r); digitalWrite(N_Y,y); digitalWrite(N_G,g);
                                   digitalWrite(S_R,r); digitalWrite(S_Y,y); digitalWrite(S_G,g); }
void setEW(bool r,bool y,bool g) { digitalWrite(E_R,r); digitalWrite(E_Y,y); digitalWrite(E_G,g);
                                   digitalWrite(W_R,r); digitalWrite(W_Y,y); digitalWrite(W_G,g); }

void applyCycle(const Cycle& c) {
  phase = c.nsFirst ? PH_NS_G : PH_EW_G;
  tStart = millis();
}

bool parseCycle(const String& s, Cycle& out) {
  // Robust sscanf parsing
  // Example: "CYCLE NS 9000 6000 2000 1000"
  char order[4] = {0};
  unsigned long ns=0, ew=0, amb=0, red=0;
  int n = sscanf(s.c_str(), "CYCLE %3s %lu %lu %lu %lu", order, &ns, &ew, &amb, &red);
  if (n != 5) return false;

  out.nsFirst = (String(order) == "NS");
  out.nsG     = (uint32_t)max(1000UL, ns);
  out.ewG     = (uint32_t)max(1000UL, ew);
  out.amber   = (uint32_t)max(500UL,  amb);
  out.allred  = (uint32_t)max(500UL,  red);
  return true;
}

void onMsg(char* topic, byte* payload, unsigned int len) {
  String s; s.reserve(len);
  for (unsigned int i=0;i<len;i++) s += (char)payload[i];
  s.trim();
  Cycle c;
  if (parseCycle(s, c)) {
    pending = c;
    hasPending = true;
    Serial.print("[cycle] "); Serial.println(s);
  } else {
    Serial.print("[warn] bad payload: "); Serial.println(s);
  }
}

void ensureMqtt() {
  while (!mqtt.connected()) {
    Serial.print("MQTT...");
    if (mqtt.connect(MQTT_CLIENT_ID)) {  // <<< UNIQUE CLIENT ID
      Serial.println("connected");
      mqtt.subscribe(SUB_TOPIC, 1);
    } else {
      Serial.print("failed rc="); Serial.print(mqtt.state()); Serial.println(" retrying...");
      delay(1000);
    }
  }
}

void setup() {
  Serial.begin(115200);

  int pins[] = {N_R,N_Y,N_G,S_R,S_Y,S_G,E_R,E_Y,E_G,W_R,W_Y,W_G};
  for (int p: pins) { pinMode(p, OUTPUT); digitalWrite(p, LOW); }

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("WiFi");
  while (WiFi.status()!=WL_CONNECTED){ delay(300); Serial.print("."); }
  Serial.println(" OK");

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(onMsg);
  ensureMqtt();

  applyCycle(active); // start defaults
}

void loop() {
  if (!mqtt.connected()) ensureMqtt();
  mqtt.loop();

  // Safe apply: switch if in ALL_RED, else at phase boundary
  if (hasPending && (phase==PH_ALLRED1 || phase==PH_ALLRED2)) {
    active = pending; hasPending=false; applyCycle(active);
  }

  uint32_t now = millis();
  uint32_t elapsed = now - tStart;

  switch (phase) {
    case PH_NS_G:
      setNS(false,false,true);  setEW(true,false,false);
      if (elapsed >= active.nsG) { phase = PH_NS_Y; tStart = now; }
      break;

    case PH_NS_Y:
      setNS(false,true,false);  setEW(true,false,false);
      if (elapsed >= active.amber) { phase = PH_ALLRED1; tStart = now; }
      break;

    case PH_ALLRED1:
      setNS(true,false,false);  setEW(true,false,false);
      if (elapsed >= active.allred) {
        phase = PH_EW_G; tStart = now;
        if (hasPending){ active=pending; hasPending=false; }
      }
      break;

    case PH_EW_G:
      setNS(true,false,false);  setEW(false,false,true);
      if (elapsed >= active.ewG) { phase = PH_EW_Y; tStart = now; }
      break;

    case PH_EW_Y:
      setNS(true,false,false);  setEW(false,true,false);
      if (elapsed >= active.amber) { phase = PH_ALLRED2; tStart = now; }
      break;

    case PH_ALLRED2:
      setNS(true,false,false);  setEW(true,false,false);
      if (elapsed >= active.allred) {
        phase = PH_NS_G; tStart = now;
        if (hasPending){ active=pending; hasPending=false; }
      }
      break;
  }
}
