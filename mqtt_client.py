import time
import paho.mqtt.client as mqtt
import logging
import psycopg2
import json
import struct
import threading
import socket

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s', handlers=[logging.StreamHandler()])
logger = logging.getLogger(__name__)

client = None

# MQTT CONFIGURATION
MQTT_CLIENT_NAME = ""
MQTT_USER = ""
MQTT_PASSWORD = ""
MQTT_HOST = ""
MQTT_PORT = 8883
MQTT_KEEPALIVE = 60
MQTT_CLEAN_SESSION = False
MQTT_QOS = 0
MQTT_TOPIC = "test"

timestamp_send = 0

def on_connect(client, userdata, flags, rc, props):
    logger.info(f"Client connected")
    client.subscribe(MQTT_TOPIC)

def on_message(client, userdata, msg):
    global timestamp_send
    
    time_recv = time.time()
    payload = msg.payload.decode()
    data_json = json.loads(payload)
    logger.info(f"Message received from {client._client_id.decode()}")
    
    if "empty" in data_json:
        logger.info("Empty message received")
        timestamp_send = time_recv
    else:
        logger.info("Data message received")
        timestamp_received = time_recv
        size = msg.payload.__sizeof__()
        logger.info(f"Message: {data_json}")
        logger.info(f"Message size: {size} bytes")

def udp_throughput_listener(stop_event: threading.Event):
    UDP_IP = "0.0.0.0"
    UDP_PORT = 5000
    NS_TO_S = 1000000000
    UDP_HEADER = 8
    IP_HEADER = 20
    ETHERNET_HEADER = 14

    # Create UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, 35, 1)
    sock.bind((UDP_IP, UDP_PORT))

    logger.info(f"[UDP Thread] Listening on UDP port {UDP_PORT}...")

    try:
        while not stop_event.is_set():
            data, ancdata, flags, addr = sock.recvmsg(4096, 1024)
            data1, ancdata1, flags, addr = sock.recvmsg(4096, 1024)

            ts_sec, ts_nsec = struct.unpack('qq', ancdata[0][2])
            timestamp_ns_1 = ts_sec * NS_TO_S + ts_nsec

            ts_sec, ts_nsec = struct.unpack('qq', ancdata1[0][2])
            timestamp_ns_2 = ts_sec * NS_TO_S + ts_nsec

            time_diff_ns = timestamp_ns_2 - timestamp_ns_1
            size_bits = (len(data1) + UDP_HEADER + IP_HEADER + ETHERNET_HEADER) * 8

            if time_diff_ns > 0:
                throughput_bps = size_bits / (time_diff_ns / NS_TO_S)
                packed = struct.pack('!f', throughput_bps)
                sock.sendto(packed, addr)
                logger.info(f"[UDP Thread] Throughput: {throughput_bps / 1000.0:.2f} kbps")
    except Exception as e:
        logger.info(f"[UDP Thread] Error: {e}")
    finally:
        sock.close()
        logger.info("[UDP Thread] Socket closed.")
        
if __name__ == "__main__":

    try:
        # Initialize MQTT client
        client = mqtt.Client(client_id=MQTT_CLIENT_NAME, callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
        client.username_pw_set(username=MQTT_USER, password=MQTT_PASSWORD)
        client.tls_set()
        client.connect(host=MQTT_HOST, port=MQTT_PORT, keepalive=MQTT_KEEPALIVE)
        client.on_message = on_message
        client.on_connect = on_connect
        
        # Set up the PIR sensor
        client.loop_start()
        
        stop_event = threading.Event()
        udp_thread = threading.Thread(target=udp_throughput_listener, args=(stop_event,))
        udp_thread.start()

        while True:
            time.sleep(5)

    except Exception as e:
        logger.exception("An error occurred")

    finally:
        logger.info("Cleaned up resources")
        stop_event.set()
        udp_thread.join()
