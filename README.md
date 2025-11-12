# Smart Adaptive Traffic Intersection — STEM Demo

> A complete, classroom-friendly project that uses **Python + OpenCV** to count cars, runs a simple **adaptive 2-phase algorithm** (NS vs EW), and controls **ESP32 traffic lights** and **line-follower smart cars** over **MQTT**.

---

## ✨ Highlights

* **Camera-based counting**: 4 manual ROIs (N/S/E/W), background subtraction + line crossing.
* **Adaptive timing**: green times 5–12 s based on live counts (short demo windows for school labs).
* **Simple IoT protocol**: one compact message for signals, tiny GO/STOP messages for cars.
* **Non-blocking FSM** on ESP32 (no `delay()`), clean pin maps, ready to breadboard.
* **Works with a free public broker**: `broker.emqx.io:1883`.

---

## 📦 Repository Layout

```
/ (root)
│
├─ vision_select_and_count.py     # OpenCV: select ROIs + count cars in N,S,E,W
├─ algo_two_phase.py              # Timing planner (NS/EW)
├─ iot_publisher.py               # Glues vision + algo; publishes MQTT to signals & cars
├─ esp32_signals_controller.ino   # ESP32: subscribes 'signals/cycle', drives 4 traffic lights
├─ esp32_car_linefollower.ino     # ESP32: car (3 IR sensors + L298N), subscribes 'cars/<dir>'
├─ roi_config.json                # Auto-generated after first ROI selection
└─ README.md                      # You’re here
```

---

## 🧠 System Overview

```
               +-----------------------------+
               |  Laptop (Python)            |
               |  - OpenCV vision            |
  Camera --->  |  - 2-phase algorithm (NS/EW)|  ---> MQTT "signals/cycle"
               |  - MQTT publisher           |  ---> MQTT "cars/<dir>" (GO/STOP)
               +-----------------------------+
                           |  Wi-Fi / MQTT (broker.emqx.io:1883)
                           v
     +---------------------------+          +-------------------+   +-------------------+   +-------------------+
     |  ESP32 Signals Controller |          | ESP32 Car (N dir) |   | ESP32 Car (S dir) |   | ESP32 Car (E dir) |
     |  4x R/Y/G traffic lights  |          | Line follower     |   | Line follower     |   | Line follower     |
     |  Non-blocking FSM         |          | (3 IR + L298N)    |   | (3 IR + L298N)    |   | (3 IR + L298N)    |
     +---------------------------+          +-------------------+   +-------------------+   +-------------------+
```

* **Counting period**: ~1 Hz (once per second).
* **Algorithm**: picks **NS or EW** to go first, computes `NS_green_ms` & `EW_green_ms` within **5–12 s**.
* **Safety**: **Amber** 2 s + **All-Red** 1 s between phases.
* **Fail-safe**: if no new data, controller falls back to a fixed cycle.

---

## 🔌 Hardware

### 1) Traffic Lights (ESP32)

* **LEDs**: 4 directions × (Red, Yellow, Green) → **12 LEDs** + 12× **220 Ω** resistors.
* **Common GND**. If you drive big LEDs or 12 V stacks, buffer with **ULN2803**.

**Default pin map (changeable at top of the `.ino`):**

| Direction | Red | Yellow | Green |
| --------- | --: | -----: | ----: |
| North     |  21 |     22 |    23 |
| South     |  19 |     18 |     5 |
| East      |  25 |     26 |    27 |
| West      |  32 |     33 |     4 |

### 2) Smart Cars (3 units, each ESP32 + L298N)

* **Motors**: DC gear motors via **L298N**.
* **Line sensors**: **3 IR** (Left / Center / Right) with a fixed threshold.
* **Power**: each car has its own battery (6–9 V) + switch.

**Default pins per car:**

| Module                   | Pin          |
| ------------------------ | ------------ |
| ENA (PWM)                | 25           |
| IN1, IN2                 | 26, 27       |
| ENB (PWM)                | 14           |
| IN3, IN4                 | 12, 13       |
| IR Left / Center / Right | 34 / 35 / 32 |

(*34/35 are input-only on ESP32 → perfect for analog IR sensors.*)

---

## 🌐 Network & MQTT

* **Broker**: `broker.emqx.io`
* **TCP Port**: `1883`
* **Auth**: none (public test broker)
* **Topics**

  * **Signals**: `signals/cycle`

    * Payload (single line):

      ```
      CYCLE <ORDER> <NS_green_ms> <EW_green_ms> <amber_ms> <allred_ms>
      ```

      Example: `CYCLE NS 9000 6000 2000 1000`
  * **Cars**: `cars/N`, `cars/S`, `cars/E`, `cars/W`

    * Payload: `GO <ms>` or `STOP`
    * Example: `GO 9000`, `STOP`

> Cars subscribe only to **their own** topic (e.g., the N-car listens to `cars/N`).

---

## 🖥️ Software Setup (Laptop)

**Requirements**

* Python 3.10+
* `pip install opencv-python numpy paho-mqtt`

**Run order**

1. `vision_select_and_count.py`

   * First run: it asks you to **select 4 ROIs** in order **N, S, E, W**.
   * It saves them into `roi_config.json`.
2. `iot_publisher.py`

   * Imports the counts (1 Hz), runs the algorithm, publishes MQTT to **signals** and **cars**.

**Quick commands**

```bash
pip install opencv-python numpy paho-mqtt
python vision_select_and_count.py    # pick ROIs & verify overlay looks right
python iot_publisher.py              # starts publishing live cycles + GO/STOP
```

---

## ⚙️ Algorithm (Short Demo Windows)

* **Target ranges**:

  * `NS_green_ms`, `EW_green_ms` ∈ **[5000 … 12000]**
  * `Amber=2000 ms`, `All-Red=1000 ms`
* **2-phase rule**:

  * Compute `NS=N+S`, `EW=E+W`; choose the larger (with small hysteresis `Δ=2 cars`).
  * Green time scales with demand (base + k×(q − average)).
* **Fairness**: hysteresis prevents flip-flop near ties; periods are short by design for demos.

---

## 🚦 Controller Logic (ESP32, non-blocking)

* Subscribes `signals/cycle`.
* Parses one line → stores a **pending** cycle.
* Applies at **safe boundary** (during All-Red), otherwise finishes the current phase first.
* Drives 12 LEDs with the FSM:

  1. `NS_G` → `NS_Y` → `ALL_RED`
  2. `EW_G` → `EW_Y` → `ALL_RED`
* If no new cycles arrive for a while, the default cycle continues.

---

## 🛤️ Smart Cars Behavior

* Subscribe to `cars/<dir>`:

  * `GO <ms>` → enable line-follower for `<ms>` (with basic left/right correction from 3 IRs)
  * `STOP` → immediate PWM=0
* Simple, robust, and perfect for a classroom demo.

---

## 🧪 First Demo Checklist

1. **Vision overlay looks correct**:

   * ROIs align with the four approaches to the intersection.
   * Counters increase when toy cars move toward the counting line.
2. **MQTT visible** (optional):

   * Use **IoT MQTT Panel** or the broker’s web console to observe:

     * `signals/cycle` messages
     * `cars/N`,`cars/S`,`cars/E`,`cars/W` GO/STOP
3. **Signals**:

   * LEDs switch **NS → amber → all-red → EW → amber → all-red** with the published durations.
4. **Cars**:

   * Start/stop per their topics.
   * Tune `PWM_SPEED` and `IR_TH` per car if needed.

---

## 🔧 Tuning Tips

* **Counting too low?**
  Increase ROI size slightly; lower `MIN_AREA`; move `LINE_POS` closer to the flow; ensure steady lighting.
* **Over-counting?**
  Increase `BAND_FRAC` slightly or position the counting line a bit further from noisy zones.
* **Line follower drifts?**
  Reduce `PWM_SPEED`, re-tape the track (solid contrast), adjust `IR_TH`.
* **Broker hiccups?**
  Public broker can drop packets—keep messages short; QoS 1 is already used where it matters.

---

## 🧰 Bill of Materials (suggested)

* 1× **ESP32 DevKit** (traffic lights)
* 3× **ESP32 DevKit** (cars)
* 3× **L298N** motor drivers
* 3× **Robot chassis kits** (2 DC motors + wheels + frame)
* 3× **Battery pack** (6–9 V) + switches
* 12× **LEDs** (R/Y/G × 4 directions)
* 12× **220 Ω** resistors
* **Jumper wires**, breadboards, common GND rails
* Printed **line track** (black line on white background)

---

## ❓ FAQ

* **Why only two phases (NS & EW)?**
  It’s the safest, simplest non-conflicting scheme for a 4-way demo with one camera. You can extend to 4-phase later.

* **Why send durations not timestamps?**
  Avoids clock sync; the controller uses relative `millis()`.

* **Can I make cars obey lane-specific signals?**
  Yes—create per-lane topics (e.g., `cars/N/left`) and widen the protocol. Start simple first.

---

## ✅ What to Edit Quickly

* In **`.ino` files**: set your `WIFI_SSID` and `WIFI_PASS`.
* In **`esp32_car_linefollower.ino`** for each car: set `CAR_TOPIC` (`cars/N`, `cars/S`, or `cars/E`) and optionally `PWM_SPEED` / `IR_TH`.

---

💬 Questions / Help

If you have any questions, want to report an issue, or need a quick hand setting things up, just message me on WhatsApp:

➡️ Chat on WhatsApp
https://wa.me/201064535868
