# LTE Gateway

This project contains Zephyr samples, documentation, and Python scripts for evaluating data collection using NB-IoT and LTE-M.

## 📦 Requirements

- Nordic nRF Connect SDK v2.9.1 (e.g., `/opt/nordic/v2.9.1`)
- nRF Connect for VS Code

---

## 1. Zephyr Sample Usage

- Open either `lwm2m_sample` or `mqtt_sample` in a separate folder using the nRF Connect VS Code extension.
- Before building, set the correct MQTT and UDP credentials in the `KConfig` file.
- **MQTT Sample:**
  - Request the `supl` folder from Nordic (via email).
  - Place the `supl` folder in the location specified in the build instructions.
  - Add your MQTT certificate to `certificates.h` in `mqtt_sample/src/certificates.h`.
- **LwM2M Sample:**
  - Place a PSK onto the Leshan demo website.
- After configuration, flash the firmware to your board.

---

## 2. Documentation

- To build the documentation, run:
  ```sh
  ./build_latex.sh
  ```
  from the root folder.

---

## 3. Python Scripts

- Install required libraries:
  ```sh
  pip install -r requirements.txt
  ```

- **Measurement Analysis:**
  - Use `analyze_lwm2m.py` or `analyze_mqtt.py` to analyze the measurements in the `Measurements` folder.

- **Serial Log Collection:**
  - Use `serial_scanner.py` to get logs from the two nrf9160dk boards.
  - Make sure the IMEI and serial numbers fit; otherwise, the serial port cannot be opened.

- **MQTT and UDP Listener:**
  - Use `mqtt_client.py` to listen to MQTT messages and start the UDP listener for throughput calculations.

---
