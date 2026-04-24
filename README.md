# 🗳️ Tri-Factor Biometric EVM (Zero-Spoof Architecture)

This repository contains the complete hardware firmware and edge-AI networking stack for an industry-grade, Tri-Factor Electronic Voting Machine (EVM). 

Designed to completely eliminate "ghost voting" and election fraud, this system physically isolates voter verification from the ballot casting process. It leverages a Master/Slave hardware configuration, robust Python concurrency, and a proprietary Just-In-Time (JIT) biometric template handshake.

## 🏗️ Dual-Node Hardware Architecture

### 1. Control Unit (CU) - Voter Verification Hub
* **Hardware:** Arduino UNO Q (SBC featuring Linux + Real-time MCU).
* **Role:** Managed by the Presiding Officer (PO). It handles the primary biometric database and network routing.
* **Firmware:** `uno_Q.ino` controls the local R307S fingerprint sensor, the 4x4 matrix keypad, and the LCD UI.
* **Edge-AI Brain:** `main.py` runs natively on the Linux OS, utilizing `DeepFace` (Facenet512) and `MediaPipe` to process live USB webcam feeds for facial recognition and liveness detection.

### 2. Ballot Unit (BU) - Vote Casting Terminal
* **Hardware:** Arduino Uno R3.
* **Role:** Placed inside the private voting booth. Operates in a stateless, heavily locked condition until authorized by the CU.
* **Storage:** Votes are tallied locally and securely on the Arduino R3's non-volatile EEPROM memory (Addresses 0, 4, 8) to completely air-gap the final ballot counts from the networked CU.
* **Firmware:** `r3.ino` manages the 3rd-party voting buttons, status LEDs, and a secondary R307S fingerprint sensor.

## 🔐 The "Tri-Factor" Authentication Protocol

To cast a single vote, the system enforces three distinct layers of biometric security:

1.  **Factor 1 (CU Fingerprint):** The voter provides their registered fingerprint at the Control Unit to initiate the session.
2.  **Factor 2 (Face Liveness & Matching):** The UNO Q triggers the USB webcam. The system normalizes lighting (CLAHE), checks for a physical blink (EAR calculation) to prevent photo spoofing, and matches the face against the SQLite database using Facenet512 embeddings.
3.  **Factor 3 (JIT Fingerprint Sync):** Once verified, the CU dynamically streams the voter's raw fingerprint template to the Ballot Unit over a TCP socket (`172.17.0.1:9000`). The voter must walk to the BU and scan their finger again to unlock the candidate buttons, ensuring the verified voter is the one physically casting the ballot.

## 🛡️ Software Reliability & Watchdogs
This version introduces a "Full Hardware Watchdog" architecture. 
* **Hardware Fault Tolerance:** The Python bridge actively monitors the camera, Master fingerprint sensor, and the R3 TCP socket. If a device disconnects, the system safely pauses and automatically resumes upon reconnection.
* **Thread-Safe Queues:** Eliminates race conditions and PING/PONG contamination over the serial bridge during active voting sessions.

## 🔌 Hardware Pin Configurations

### Control Unit (Arduino UNO Q)
* **Fingerprint (Master):** Hardware Serial Pins `0 (RX)` / `1 (TX)`. *(Note: Uses a voltage divider as the R307S logic operates at 3.3V).*
* **Matrix Keypad:** Rows `2, 3, 4, 5` | Columns `A2, A3, A4, A5`.
* **16x2 LCD:** I2C (SDA/SCL).

### Ballot Unit (Arduino Uno R3)
* **Fingerprint (Slave):** SoftwareSerial `2 (RX)` / `3 (TX)`. *(Note: Uses a voltage divider as the R307S logic operates at 3.3V).*
* **Party LED Indicators:** Pins `9`, `10`, `11` (PWM capable for dynamic chase effects).
* **EEPROM Vote Addresses:** Party A (`0`), Party B (`4`), Party C (`8`).

## 🚀 Deployment Instructions

1.  **Flash Firmware:** Upload `uno_Q.ino` to the UNO Q MCU and `r3.ino` to the Uno R3.
2.  **Hardware Connection:** Ensure the Uno R3 is connected to the UNO Q via USB.
3.  **Launch the AI & Network Bridge:** Open the UNO Q's Linux terminal and execute the following commands to activate the Conda environment, bind the serial-to-TCP socket, and start the daemon:
    ```bash
    source ~/miniforge3/bin/activate
    conda activate evm_vision
    socat TCP-LISTEN:9000,fork,reuseaddr FILE:/dev/ttyACM0,b115200,raw,echo=0,nonblock,waitlock=/tmp/ttyACM0.lock &
    python3 main.py
    ```
4.  **Admin Initialization:** If the database is empty, the system will prompt the Presiding Officer (ID: 1) to enroll their fingerprint and face. This Master ID is strictly required to authorize voting sessions, wipe EEPROM tallies, and view final results.
