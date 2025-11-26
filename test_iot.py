# ==============
# iot_publisher_hi.py
# ==============
# Simple MQTT publisher to test connection.
# Publishes:
#   - signals/cycle  -> "HI"
#
# Requirements:
#   pip install paho-mqtt

from __future__ import annotations

import time
import uuid
import traceback
from typing import Dict

import paho.mqtt.client as mqtt

# ---------- MQTT settings ----------
BROKER_HOST = "broker.emqx.io"
BROKER_PORT = 1883

# A readable prefix; we add a random suffix to avoid collisions across laptops/instances.
CLIENT_ID_PREFIX = "laptop_traffic_pub"

TOPIC_CYCLE = "signals/cycle"
TOPIC_CARS: Dict[str, str] = {
    "N": "cars/N",
    "S": "cars/S",
    "E": "cars/E",
    "W": "cars/W",
}

# ---------- Reconnect/backoff helpers ----------
def ensure_connected(client: mqtt.Client, max_backoff: int = 10) -> bool:
    """
    Ensure the client is connected; try reconnect with incremental backoff.
    Returns True when connected, False if a fatal error occurred.
    """
    if client.is_connected():
        return True

    backoff = 1
    while not client.is_connected() and backoff <= max_backoff:
        try:
            # In paho, reconnect() is used after loop_start()
            rc = client.reconnect()
            if rc == mqtt.MQTT_ERR_SUCCESS:
                return True
            print(f"[mqtt] reconnect rc={rc}; retrying in {backoff}s")
        except Exception as e:
            print(f"[mqtt] reconnect exception: {e}; retrying in {backoff}s")
        time.sleep(backoff)
        backoff = min(max_backoff, backoff * 2)

    return client.is_connected()


def safe_publish(
    client: mqtt.Client,
    topic: str,
    payload: str,
    qos: int = 1,
    retain: bool = False,
) -> bool:
    """
    Publish with error checks. Returns True on success, False otherwise.
    """
    try:
        info = client.publish(topic, payload, qos=qos, retain=retain)
        # info.rc is immediate status; wait_for_publish ensures QoS handshake locally
        if info.rc != mqtt.MQTT_ERR_SUCCESS:
            print(f"[mqtt] publish rc={info.rc} topic={topic}")
            return False
        info.wait_for_publish(timeout=3.0)
        if not info.is_published():
            print(f"[mqtt] not published within timeout topic={topic}")
            return False
        return True
    except Exception as e:
        print(f"[mqtt] publish exception on {topic}: {e}")
        return False


# ---------- MQTT callbacks (optional logs) ----------
def _on_connect(
    client: mqtt.Client,
    userdata,
    flags,
    rc,
    properties=None,
) -> None:
    print(f"[mqtt] connected rc={rc}")


def _on_disconnect(
    client: mqtt.Client,
    userdata,
    rc,
    properties=None,
) -> None:
    print(f"[mqtt] disconnected rc={rc}")


# ---------- Core publishing ----------
def _publish_hi(client: mqtt.Client) -> bool:
    """
    Publish simple 'HI' message to signals/cycle and log result.
    """
    msg = "hiiiiiii"
    ok1 = safe_publish(client, TOPIC_CYCLE, msg, qos=1, retain=False)
    print(("[signals] OK  " if ok1 else "[signals] FAIL"), msg)

    # (اختياري) تقدر كمان تبعت HI على توبكس العربيات لو حابب الاختبار يكون أوسع:
    # okN = safe_publish(client, TOPIC_CARS["N"], "HI", qos=1)
    # okS = safe_publish(client, TOPIC_CARS["S"], "HI", qos=1)
    # okE = safe_publish(client, TOPIC_CARS["E"], "HI", qos=1)
    # okW = safe_publish(client, TOPIC_CARS["W"], "HI", qos=1)

    return ok1


def main() -> None:
    # Build unique client id to avoid broker collisions if you run multiple instances.
    unique_id = f"{CLIENT_ID_PREFIX}_{uuid.uuid4().hex[:6]}"
    client = mqtt.Client(client_id=unique_id, clean_session=True)
    client.on_connect = _on_connect
    client.on_disconnect = _on_disconnect

    # First connect (with robust error handling)
    try:
        rc = client.connect(BROKER_HOST, BROKER_PORT, keepalive=30)
        if rc != mqtt.MQTT_ERR_SUCCESS:
            print(f"[mqtt] connect failed rc={rc}")
            return
    except Exception:
        print("[mqtt] connect exception:")
        traceback.print_exc()
        return

    # Network thread
    client.loop_start()

    try:
        # Simple loop: send "HI" every 5 seconds
        while True:
            # Ensure MQTT is connected before publishing
            if not ensure_connected(client):
                print("[mqtt] unable to reconnect; skipping this tick")
                time.sleep(2.0)
                continue

            _publish_hi(client)
            time.sleep(5.0)

    except KeyboardInterrupt:
        print("\n[main] interrupted by user")
    except Exception:
        print("[main] exception:")
        traceback.print_exc()
    finally:
        try:
            client.loop_stop()
            client.disconnect()
        except Exception:
            pass
        print("[main] clean exit")


if __name__ == "__main__":
    main()
