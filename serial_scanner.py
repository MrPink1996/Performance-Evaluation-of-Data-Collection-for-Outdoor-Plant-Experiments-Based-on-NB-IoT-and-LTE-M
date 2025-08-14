import serial
import logging
import threading
from serial.tools import list_ports
from dataclasses import dataclass
import queue
import os
import json
import time
import base64
from scapy.utils import PcapWriter
from scapy.packet import Raw
import sys
import subprocess
import requests
from urllib.parse import quote
import signal
import paho.mqtt.client as mqtt
import platform
import datetime

RESOURCE_NETWORK_BEAERER = "4/0/0"
RESOURCE_AVAILABLE_NETWORK_BEARER = "4/0/1"
RESOURCE_RADIO_SIGNAL_STRENGTH = "4/0/2"
RESOURCE_LINK_QUALITY = "4/0/3"
RESOURCE_IP_ADDRESS = "4/0/4"
RESOURCE_ROUTER_IP_ADDRESS = "4/0/5"
RESOURCE_LINK_UTILIZATION = "4/0/6"
RESOURCE_APN = "4/0/7"
RESOURCE_CELL_ID = "4/0/8"
RESOURCE_SMNC = "4/0/9"
RESOURCE_SMCC = "4/0/10"

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)    

thread_queue = queue.Queue()
stop_trace_lwm2m = threading.Event()
stop_trace_mqtt = threading.Event()
start_trace_lwm2m = threading.Event()
start_trace_mqtt = threading.Event()

@dataclass
class nrf_device:
    name: str
    serial_number: int
    port_vcom_1: str
    port_vcom_2: str
    port_vcom_3: str
    imei: int


if platform.system().lower() == "linux":
    GPS_DEVICE = nrf_device(name="GPS Device", serial_number=960018448 ,port_vcom_1="/dev/ttyACM0", port_vcom_2="/dev/ttyACM1", port_vcom_3="/dev/ttyACM2", imei=351358815320228)
    MQTT_DEVICE = nrf_device(name="MQTT Device", serial_number=960026788 ,port_vcom_1="/dev/ttyACM0", port_vcom_2="/dev/ttyACM1", port_vcom_3="/dev/ttyACM2", imei=350457790974313)
    LWM2M_DEVICE = nrf_device(name="LWM2M Device", serial_number=960090316 ,port_vcom_1="/dev/ttyACM0", port_vcom_2="/dev/ttyACM1", port_vcom_3="/dev/ttyACM2", imei=351358815327405)
else:
    GPS_DEVICE = nrf_device(name="GPS Device", serial_number=960018448 ,port_vcom_1="/dev/cu.usbmodem0009600184481", port_vcom_2="/dev/cu.usbmodem0009600184483", port_vcom_3="/dev/cu.usbmodem0009600184485", imei=351358815320228)
    MQTT_DEVICE = nrf_device(name="MQTT Device", serial_number=960026788 ,port_vcom_1="/dev/cu.usbmodem0009600267881", port_vcom_2="/dev/cu.usbmodem0009600267883", port_vcom_3="/dev/cu.usbmodem0009600267885", imei=350457790974313)
    LWM2M_DEVICE = nrf_device(name="LWM2M Device", serial_number=960090316 ,port_vcom_1="/dev/cu.usbmodem0009600903161", port_vcom_2="/dev/cu.usbmodem0009600903163", port_vcom_3="/dev/cu.usbmodem0009600903165", imei=351358815327405)

def get_measurement_number():
    """Get the number of measurements by checking the folder structure of the Measurements folder. Sub measurements are stored in the Measurements/measurement_id folder. """
    
    dirs = [d for d in os.listdir('Measurements') if os.path.isdir(os.path.join('Measurements', d))]
    
    count = -1
    for dir in dirs:
        measurement_id = dir.split('_')[1]
        if measurement_id.isdigit():
            count = max(count, int(measurement_id))
    
    return count + 1

def read_resource(resource_path, DEVICE_ID):
    DEVICE_ID = f"urn:imei:{DEVICE_ID}"  # Ensure DEVICE_ID is in the correct format
    encoded_id = quote(DEVICE_ID, safe='')  # Only encode device ID

    # Correct URL without encoding path
    url = f"https://leshan.eclipseprojects.io/api/clients/{encoded_id}/{resource_path}"
    response = requests.get(url)   
    # print(f"🔍 Reading resource {resource_path} from {url}")
    # print(f"🔍 Response status code: {response.status_code}")

    if response.status_code != 200:
        print(f"❌ Error reading resource {resource_path}: {response.status_code}")
        return False, None
    
    data = response.json()
    # print(f"🔍 Response data for {resource_path}: {data}")
    if "delayed" in data.keys():
        print("🔄 Resource is delayed, retrying...")
        return False, None
    

    if data['status'] != "CONTENT(205)":
        print(f"❌ Resource {resource_path} not available or not readable: {data['status']}")
        return False, None
    
    content = data['content']
    # print(f"🔍 Content for {resource_path}: {content}")
    
    if content['kind'] == 'singleResource':
        return True, content['value']
    elif content['kind'] == 'multiResource':
        if content['type'] == "INTEGER":
            return True, [int(item) for item in content['values'].values()]
        if content['type'] == "STRING":
            return True, [str(item) for item in content['values'].values()]

    return False, response

def wait_for_port(port_name, timeout=30, interval=0.5):
    """Wait until the specified serial port appears or timeout expires."""
    start_time = time.time()
    while True:
        ports = [p.device for p in serial.tools.list_ports.comports()]
        if port_name in ports:
            return True
        if time.time() - start_time > timeout:
            return False
        time.sleep(interval)

def print_serial_ports():
    """Scan for available serial ports."""
    ports = list_ports.comports()
    available_ports = []
    for port in ports:
        print(f"Port: {port.device}, Description: {port.description}, HWID: {port.hwid}")
    
def get_gps_data():
    """Read logs from gps device and extract location data."""
    """Add longitude, latitude and accuracy to the thread queue."""
    if wait_for_port(GPS_DEVICE.port_vcom_3):
        ser = serial.Serial(GPS_DEVICE.port_vcom_3, baudrate=115200, timeout=1)
        logger.info(f"Connected to GPS device on port {GPS_DEVICE.port_vcom_3}")
    full_log = ""
    try:
        while True:
            if ser.in_waiting:
                try: 
                    line = ser.readline().decode('utf-8')
                    full_log += time.ctime() + " : " + line

                    if "Connected to the network in mode: 7" in line:
                        logger.info("GPS: Connected to the network in mode: 7")
                    if "Connected to the network in mode: 9" in line:
                        logger.info("GPS: Connected to the network in mode: 9")

                    if "GNSS started, waiting for fix..." in line:
                        logger.info("GPS: Start gnss and wait for fix")

                    if "SUPL session finished" in line:
                        logger.info("GPS: SUPL session finished")

                    if "FOUND GNSS FIX" in line:
                        latitude = float(line.split("Latitude:")[1].split(",")[0])
                        longitude = float(line.split("Longitude:")[1].split(",")[0])
                        accuracy = float(line.split("Accuracy:")[1].split(",")[0])
                        data = {
                            "device": GPS_DEVICE.name,
                            "serial_number": GPS_DEVICE.serial_number,
                            "imei": GPS_DEVICE.imei,
                            "vcom_port": 3,
                            "latitude": latitude,
                            "longitude": longitude,
                            "accuracy": accuracy,
                            "log": full_log
                        }
                        thread_queue.put(data)

                    if f"PROGRAMM FINISHED" in line:
                        logger.info("End of GPS log detected.")
                        break

                except Exception as e:
                    logger.error(f"Error decoding line from serial port: {e}")
                    continue
    except Exception as e:
        logger.error(f"Error decoding line from serial port: {e}")

def get_lwm2m_log():
    """Get LWM2M log from the lwm2m device."""
    """Add log data to the thread queue."""

    if wait_for_port(LWM2M_DEVICE.port_vcom_1):
        ser = serial.Serial(LWM2M_DEVICE.port_vcom_1, baudrate=115200, timeout=1)
        logger.info(f"Connected to LWM2M device on port {LWM2M_DEVICE.port_vcom_1}")
        start_trace_lwm2m.set()


    full_log = ""
    timestamp_nbiot = 0
    timestamp_ltem = 0
    try:
        while True:
            if ser.in_waiting:
                try: 
                    line = ser.readline().decode('utf-8')
                    if "app_lwm2m_client: LwM2M is connected to server" in line:
                        logger.info("LWM2M client is connected to server, reading resources...")
                        tic = time.time()
                        result, buffer = read_resource(RESOURCE_NETWORK_BEAERER, LWM2M_DEVICE.imei)
                        logger.info(f"Result: {result} | Data: {buffer}")
                        result, buffer = read_resource(RESOURCE_AVAILABLE_NETWORK_BEARER, LWM2M_DEVICE.imei)
                        logger.info(f"Result: {result} | Data: {buffer}")
                        result, buffer = read_resource(RESOURCE_RADIO_SIGNAL_STRENGTH, LWM2M_DEVICE.imei)
                        logger.info(f"Result: {result} | Data: {buffer}")
                        result, buffer = read_resource(RESOURCE_LINK_QUALITY, LWM2M_DEVICE.imei)
                        logger.info(f"Result: {result} | Data: {buffer}")
                        result, buffer = read_resource(RESOURCE_IP_ADDRESS, LWM2M_DEVICE.imei)
                        logger.info(f"Result: {result} | Data: {buffer}")
                        result, buffer = read_resource(RESOURCE_APN, LWM2M_DEVICE.imei)
                        logger.info(f"Result: {result} | Data: {buffer}")
                        result, buffer = read_resource(RESOURCE_CELL_ID, LWM2M_DEVICE.imei)
                        logger.info(f"Result: {result} | Data: {buffer}")
                        result, buffer = read_resource(RESOURCE_SMNC, LWM2M_DEVICE.imei)
                        logger.info(f"Result: {result} | Data: {buffer}")
                        result, buffer = read_resource(RESOURCE_SMCC, LWM2M_DEVICE.imei)
                        logger.info(f"Result: {result} | Data: {buffer}")
                        logger.info(f"Time needed to read resources: {time.time() - tic}")
                    
                    if "Connected to the network in mode: 7" in line:
                        logger.info("LWM2M: Connected to the network in mode: 7")
                        timestamp_ltem = time.time()

                    if "Connected to the network in mode: 9" in line:
                        logger.info("LWM2M: Connected to the network in mode: 9")
                        timestamp_nbiot = time.time()

                    if f"PROGRAMM FINISHED" in line:
                        logger.info("End of LWM2M log detected.")
                        break
                        
                except Exception as e:
                    logger.error(f"Error decoding line from serial port: {e}")
                    continue
                full_log += time.ctime() + " : " + line
                #logger.info(line)  # Log the line read from the serial port
            
    except Exception as e:
        logger.error(f"Serial error: {e}")
    finally:
        ser.close()
        logger.info(f"Serial port {LWM2M_DEVICE.port_vcom_1} closed.")
        data = {
            "device": LWM2M_DEVICE.name,
            "serial_number": LWM2M_DEVICE.serial_number,
            "imei": LWM2M_DEVICE.imei,
            "vcom_port": 1,
            "log": full_log,
            "timestamp_ltem": timestamp_ltem,
            "timestamp_nbiot": timestamp_nbiot
        }
        thread_queue.put(data)

def get_lwm2m_trace():
    """Get LWM2M trace from the device."""
    """Add trace data to the thread queue."""
    while not start_trace_lwm2m.is_set():
        time.sleep(0.1)

    logger.info("Starting modem trace collection...")
    cmd = f"nrfutil trace lte --input-serialport {LWM2M_DEVICE.port_vcom_3} --output-raw lwm2m_live_trace.bin > /dev/null 2>&1"
    subprocess.Popen(cmd, shell=True)
    try:
        while not stop_trace_lwm2m.is_set():
            time.sleep(0.5)
        logger.info("Stopping LWM2M trace collection...")
    finally:
        logger.info("Stopping modem trace collection...")
        try:
            os.system("pkill -f 'nrfutil'")
            logger.info("Trace collection process terminated successfully.")
        except subprocess.TimeoutExpired:
            logger.warning("Trace collection process did not terminate in time, killing it.")
        logger.info("Trace collection process terminated.")
        os.system("nrfutil trace lte --input-file lwm2m_live_trace.bin --output-pcapng lwm2m_trace_file.pcapng")
        data = {
            "device": LWM2M_DEVICE.name,
            "serial_number": LWM2M_DEVICE.serial_number,
            "imei": LWM2M_DEVICE.imei,
            "vcom_port": 3,
        }
        thread_queue.put(data)

def get_mqtt_log():
    """Get MQTT log from the device."""
    """Add log data to the thread queue."""

    if wait_for_port(MQTT_DEVICE.port_vcom_1):
        ser = serial.Serial(MQTT_DEVICE.port_vcom_1, baudrate=115200, timeout=1)
        logger.info(f"Connected to MQTT_DEVICE device on port {MQTT_DEVICE.port_vcom_1}")
        start_trace_mqtt.set()


    full_log = ""
    timestamp_nbiot = 0
    timestamp_ltem = 0
    latitude = 0.0
    longitude = 0.0
    accuracy = 0.0
    try:
        while True:
            if ser.in_waiting:
                try: 
                    line = ser.readline().decode('utf-8')
                    if "Connected to the network in mode: 7" in line:
                        logger.info("MQTT: Connected to the network in mode: 7")
                        timestamp_ltem = time.time()

                    if "Connected to the network in mode: 9" in line:
                        logger.info("MQTT: Connected to the network in mode: 9")
                        timestamp_nbiot = time.time()

                    if "GNSS started, waiting for fix..." in line:
                        logger.info("GPS: Start gnss and wait for fix")

                    if "SUPL session finished" in line:
                        logger.info("GPS: SUPL session finished")

                    if "FOUND GNSS FIX" in line:
                        latitude = float(line.split("Latitude:")[1].split(",")[0])
                        longitude = float(line.split("Longitude:")[1].split(",")[0])
                        accuracy = float(line.split("Accuracy:")[1].split(",")[0])
                        logger.info(f"GPS: Latitude: {latitude}, Longitude: {longitude}, Accuracy: {accuracy}")

                    if "<inf> lte_ble_gw: mqtt_publish:" in line:
                        logger.info(f"MQTT:{line.split("lte_ble_gw:")[1]}")

                    if f"PROGRAMM FINISHED" in line:
                        logger.info("End of MQTT log detected.")
                        break

                except Exception as e:
                    logger.error(f"Error decoding line from serial port: {e}")
                    continue
                full_log += time.ctime() + " : " + line
                #logger.info(line)
            
    except Exception as e:
        logger.error(f"Serial error: {e}")
    finally:
        ser.close()
        logger.info(f"Serial port {MQTT_DEVICE.port_vcom_1} closed.")
        data = {
            "device": MQTT_DEVICE.name,
            "serial_number": MQTT_DEVICE.serial_number,
            "imei": MQTT_DEVICE.imei,
            "vcom_port": 1,
            "log": full_log,
            "timestamp_ltem": timestamp_ltem,
            "timestamp_nbiot": timestamp_nbiot,
            "latitude": latitude,
            "longitude": longitude,
            "accuracy": accuracy,
        }
        thread_queue.put(data)

def get_mqtt_trace():
    """Get MQTT trace from the device."""
    """Add trace data to the thread queue."""
    while not start_trace_mqtt.is_set():
        time.sleep(0.1)

    cmd = f"nrfutil trace lte --input-serialport {MQTT_DEVICE.port_vcom_3} --output-raw mqtt_live_trace.bin > /dev/null 2>&1"
    logger.info("Starting modem trace collection...")
    subprocess.Popen(cmd, shell=True)
    try:
        while not stop_trace_mqtt.is_set():
            time.sleep(0.5)
        logger.info("Stopping MQTT trace collection...")
    finally:
        logger.info("Stopping modem trace collection...")
        try:
            os.system("pkill -f 'nrfutil'")
            logger.info("Trace collection process terminated successfully.")
        except subprocess.TimeoutExpired:
            logger.warning("Trace collection process did not terminate in time, killing it.")
        logger.info("Trace collection process terminated.")
        os.system("nrfutil trace lte --input-file mqtt_live_trace.bin --output-pcapng mqtt_trace_file.pcapng")
        data = {
            "device": MQTT_DEVICE.name,
            "serial_number": MQTT_DEVICE.serial_number,
            "imei": MQTT_DEVICE.imei,
            "vcom_port": 3,
        }
        thread_queue.put(data)

def extract_data_from_log(log):
    """Extract relevant data from the log."""
    # This function should parse the log and extract relevant data.
    data = {}
    for line in log.splitlines():
        if "Connected to the network in mode: " in line:
            data['Network Mode'] = int(line.split("Connected to the network in mode: ")[1])
        elif "APN:" in line:
            data['APN'] = line.split("APN: ")[1].strip()
        elif "TAC:" in line:
            data['TAC'] = line.split("TAC: ")[1].strip()
        elif "MCC:" in line:
            data['MCC'] = int(line.split("MCC: ")[1])
        elif "MNC:" in line:
            data['MNC'] = int(line.split("MNC: ")[1])
        elif "Band:" in line:
            data['Band'] = int(line.split("Band: ")[1])
        elif "Earfcn:" in line:
            data['Earfcn'] = int(line.split("Earfcn: ")[1])
        elif ": Cell ID:" in line:
            data['Cell ID'] = int(line.split("Cell ID: ")[1])
        elif ": Phy Cell ID:" in line:
            data['Phy Cell ID'] = int(line.split("Phy Cell ID: ")[1])
        elif "Tx Power:" in line:
            data['Tx Power'] = int(line.split("Tx Power: ")[1])
        elif "Rx Repetitions:" in line:
            data['Rx Repetitions'] = int(line.split("Rx Repetitions: ")[1])
        elif "Tx Repetitions:" in line:
            data['Tx Repetitions'] = int(line.split("Tx Repetitions: ")[1])
        elif "Energy Estimate:" in line:
            data['Energy Estimate'] = int(line.split("Energy Estimate: ")[1])
        elif "Active Time:" in line:
            data['Active Time'] = int(line.split("Active Time: ")[1])
        elif "CE Level:" in line:
            data['CE Level'] = int(line.split("CE Level: ")[1])
        elif "eDRX:" in line:
            data['eDRX'] = float(line.split("eDRX: ")[1])
        elif "RSRP:" in line:
            data['RSRP'] = int(line.split("RSRP: ")[1])
        elif "RSRQ:" in line:
            data['RSRQ'] = int(line.split("RSRQ: ")[1])
        elif "SNR:" in line:
            data['SNR'] = int(line.split("SNR: ")[1])
        elif "DL Pathloss:" in line:
            data['DL Pathloss'] = int(line.split("DL Pathloss: ")[1])
        elif "Tau:" in line:
            data['Tau'] = int(line.split("Tau: ")[1])
        elif "TAU triggered:" in line:
            data['TAU triggered'] = int(line.split("TAU triggered: ")[1])
        elif "Link:" in line:
            data['Link'] = int(line.split("Link: ")[1])
        elif "Link throughput:" in line:
            data['Link throughput'] = float(line.split("Link throughput: ")[1])
    return data

def extract_time_from_log(log, keyword):
    """Extract time from the log based on a keyword."""
    for line in log.splitlines():
        if keyword in line:
            # Extract the time value after the keyword
            time = float(line.split(keyword)[1])
            return time
    return None

if __name__ == "__main__":
    # thread_get_gps = threading.Thread(target=get_gps_data)
    thread_get_lwm2m_log = threading.Thread(target=get_lwm2m_log)
    thread_get_lwm2m_trace = threading.Thread(target=get_lwm2m_trace)
    thread_get_mqtt_log = threading.Thread(target=get_mqtt_log)
    thread_get_mqtt_trace = threading.Thread(target=get_mqtt_trace)

    # Start the threads
    # thread_get_gps.start()
    thread_get_lwm2m_log.start()
    thread_get_lwm2m_trace.start()
    thread_get_mqtt_log.start()
    thread_get_mqtt_trace.start()

    # Wait for all threads to complete
    # thread_get_gps.join()
    thread_get_lwm2m_log.join()
    thread_get_mqtt_log.join()

    stop_trace_lwm2m.set()  # Signal to stop LWM2M trace
    stop_trace_mqtt.set()  # Signal to stop MQTT trace
    time.sleep(2)
    thread_get_lwm2m_trace.join()
    thread_get_mqtt_trace.join()
    
    logger.info("All threads have completed execution.")

    

    if not os.path.exists("Measurements"):
        os.makedirs("Measurements")
        logger.info("Created Measurements directory.")
    measurement_number = get_measurement_number()
    # Create a directory for the measurement
    os.makedirs(f"Measurements/measurement_{measurement_number}", exist_ok=True)
    print("Data in thread queue:")

    lwm2m_data_ltem = {}
    lwm2m_data_nbiot = {}
    mqtt_data_ltem = {}
    mqtt_data_nbiot = {}
    longitude = 0.0
    latitude = 0.0
    accuracy = 0.0

    while not thread_queue.empty():
        data = thread_queue.get()

        # PARSE LWM2M LOGS
        if data['device'] == LWM2M_DEVICE.name and data['vcom_port'] == 1:
            start_ltem = data['log'].find("Connected to the network in mode: 7")
            start_nbiot = data['log'].find("Connected to the network in mode: 9")
            data_ltem = extract_data_from_log(data['log'][start_ltem:start_nbiot])
            data_nbiot = extract_data_from_log(data['log'][start_nbiot:])
            lwm2m_data_ltem  = data_ltem
            lwm2m_data_nbiot = data_nbiot
            lwm2m_data_ltem['timestamp_ltem'] = data['timestamp_ltem']
            lwm2m_data_nbiot['timestamp_nbiot'] = data['timestamp_nbiot']
            lwm2m_data_ltem['Connection Time Link'] = extract_time_from_log(data['log'], "CONNECTION_TIME_LINK_LTEM: ")
            lwm2m_data_ltem['Connection Time Protocol'] = extract_time_from_log(data['log'], "CONNECTION_TIME_PROTOCOL_LTEM: ")
            lwm2m_data_nbiot['Connection Time Link'] = extract_time_from_log(data['log'], "CONNECTION_TIME_LINK_NBIOT: ")
            lwm2m_data_nbiot['Connection Time Protocol'] = extract_time_from_log(data['log'], "CONNECTION_TIME_PROTOCOL_NBIOT: ")

            # Save LWM2M log to file
            with open(f"Measurements/measurement_{measurement_number}/lwm2m_log.txt", "w") as f:
                f.write(data['log'])
            logger.info(f"LWM2M log saved to Measurements/measurement_{measurement_number}/lwm2m_log.txt")
        
        # PARSE LWM2M TRACE
        elif data['device'] == LWM2M_DEVICE.name and data['vcom_port'] == 3:
            # Shift both the lwm2m trace-live.bin and the lwm2m trace-file.pcapng file to the measurement folder 
            os.rename("lwm2m_live_trace.bin", f"Measurements/measurement_{measurement_number}/lwm2m_trace_live.bin")
            os.rename("lwm2m_trace_file.pcapng", f"Measurements/measurement_{measurement_number}/lwm2m_trace.pcapng")
            logger.info(f"LWM2M trace saved to Measurements/measurement_{measurement_number}")
        
        # PARSE MQTT LOGS
        elif data['device'] == MQTT_DEVICE.name and data['vcom_port'] == 1:

            data_format = data['log'].split("lte_ble_gw: FOUND GNSS FIX:")[1]
            start_ltem = data_format.find("Connected to the network in mode: 7")
            start_nbiot = data_format.find("Connected to the network in mode: 9")

            data_ltem = extract_data_from_log(data_format[start_ltem:start_nbiot])
            data_nbiot = extract_data_from_log(data_format[start_nbiot:])
            
            mqtt_data_ltem = data_ltem
            mqtt_data_nbiot = data_nbiot
            mqtt_data_ltem['timestamp_ltem'] = data['timestamp_ltem']
            mqtt_data_nbiot['timestamp_nbiot'] = data['timestamp_nbiot']
            mqtt_data_ltem['Connection Time Link'] = extract_time_from_log(data['log'], "CONNECTION_TIME_LINK_LTEM: ")
            mqtt_data_ltem['Connection Time Protocol'] = extract_time_from_log(data['log'], "CONNECTION_TIME_PROTOCOL_LTEM: ")
            mqtt_data_nbiot['Connection Time Link'] = extract_time_from_log(data['log'], "CONNECTION_TIME_LINK_NBIOT: ")
            mqtt_data_nbiot['Connection Time Protocol'] = extract_time_from_log(data['log'], "CONNECTION_TIME_PROTOCOL_NBIOT: ")
            

            latitude = data['latitude']
            longitude = data['longitude']
            accuracy = data['accuracy']
            logger.info(f"GPS data received: Latitude: {latitude}, Longitude: {longitude}, Accuracy: {accuracy}")


            # Save LWM2M log to file
            with open(f"Measurements/measurement_{measurement_number}/mqtt_log.txt", "w") as f:
                f.write(data['log'])
            logger.info(f"MQTT log saved to Measurements/measurement_{measurement_number}/mqtt_log.txt")

        # PARSE MQTT TRACE    
        elif data['device'] == MQTT_DEVICE.name and data['vcom_port'] == 3:
            # Shift both the lwm2m trace-live.bin and the lwm2m trace-file.pcapng file to the measurement folder 
            os.rename("mqtt_live_trace.bin", f"Measurements/measurement_{measurement_number}/mqtt_trace_live.bin")
            os.rename("mqtt_trace_file.pcapng", f"Measurements/measurement_{measurement_number}/mqtt_trace.pcapng")
            logger.info(f"MQTT trace saved to Measurements/measurement_{measurement_number}")

        # PARSE GPS DATA
        elif data['device'] == GPS_DEVICE.name:
            latitude = data['latitude']
            longitude = data['longitude']
            accuracy = data['accuracy']
            logger.info(f"GPS data received: Latitude: {latitude}, Longitude: {longitude}, Accuracy: {accuracy}")
            # Save LWM2M log to file
            with open(f"Measurements/measurement_{measurement_number}/gps_log.txt", "w") as f:
                f.write(data['log'])
            logger.info(f"GPS log saved to Measurements/measurement_{measurement_number}/gps_log.txt")

    ## Publish the messages and save them to the database
    save_data = {
        "measurement_number": measurement_number,
        "location_latitude": latitude,
        "location_longitude": longitude,
        "location_accuracy": accuracy,
        "lwm2m_data_ltem": lwm2m_data_ltem,
        "lwm2m_data_nbiot": lwm2m_data_nbiot,
        "mqtt_data_ltem": mqtt_data_ltem,
        "mqtt_data_nbiot": mqtt_data_nbiot
    }
    with open(f"Measurements/measurement_{measurement_number}/all_data.json", "w") as f:
        json.dump(save_data, f, indent=2)

    time.sleep(1)
    logger.info("All data processed and saved.")
