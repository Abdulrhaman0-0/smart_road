# ==============
# iot_publisher.py
# ==============
# Orchestrates Vision + Algorithm + MQTT publishing.
# Publishes:
#   - signals/cycle  -> "CYCLE <ORDER> <NS> <EW> <AMBER> <ALLRED>"
#   - cars/N,S,E,W   -> "GO <ms>" or "STOP"

import paho.mqtt.client as mqtt
from vision_select_and_count import count_stream
from algo_two_phase import plan_cycle

# ---------- MQTT settings ----------
BROKER_HOST = "broker.emqx.io"
BROKER_PORT = 1883
TOPIC_CYCLE = "signals/cycle"
TOPIC_CARS  = {
    "N": "cars/N",
    "S": "cars/S",
    "E": "cars/E",
    "W": "cars/W",
}
CLIENT_ID   = "laptop_traffic_pub"

def _publish_cycle(client, cyc, last_order):
    # 1) publish cycle line for signals
    msg = f"CYCLE {cyc.order} {cyc.ns_green_ms} {cyc.ew_green_ms} {cyc.amber_ms} {cyc.allred_ms}"
    client.publish(TOPIC_CYCLE, msg, qos=1, retain=False)
    print("[signals] ", msg)

    # 2) publish GO/STOP for cars
    if cyc.order == "NS":
        ns_ms, ew_ms = cyc.ns_green_ms, cyc.ew_green_ms
        # NS roads GO, EW STOP (cars stop until their turn)
        client.publish(TOPIC_CARS["N"], f"GO {ns_ms}", qos=1)
        client.publish(TOPIC_CARS["S"], f"GO {ns_ms}", qos=1)
        client.publish(TOPIC_CARS["E"], "STOP", qos=1)
        client.publish(TOPIC_CARS["W"], "STOP", qos=1)
    else:
        ns_ms, ew_ms = cyc.ns_green_ms, cyc.ew_green_ms
        client.publish(TOPIC_CARS["E"], f"GO {ew_ms}", qos=1)
        client.publish(TOPIC_CARS["W"], f"GO {ew_ms}", qos=1)
        client.publish(TOPIC_CARS["N"], "STOP", qos=1)
        client.publish(TOPIC_CARS["S"], "STOP", qos=1)

    return cyc.order  # new last_order

def main():
    client = mqtt.Client(client_id=CLIENT_ID, clean_session=True)
    client.connect(BROKER_HOST, BROKER_PORT, keepalive=30)
    client.loop_start()

    last_order = "NS"
    for counts in count_stream():
        # counts: {'N':..,'S':..,'E':..,'W':..}
        cyc = plan_cycle(counts["N"], counts["S"], counts["E"], counts["W"], last_order=last_order)
        last_order = _publish_cycle(client, cyc, last_order)
        # we publish once per second; the ESP32 does the precise timing

if __name__ == "__main__":
    main()
