# GymBar: MPU6050-based Barbell Path Tracker

This project uses an ESP32-C3 and an MPU6050 inertial measurement unit (IMU) to track the roll and pitch of a barbell during lifts. The data is transmitted wirelessly to a second ESP32, which can then be visualized in real-time on a computer.

## Features

*   **Real-time Roll and Pitch Tracking:** Measures the orientation of the barbell using an MPU6050.
*   **Wireless Data Transmission:** Uses the ESP-NOW protocol for low-latency, wireless communication between two ESP32 devices.
*   **Data Visualization:** Includes a Python script to plot the roll and pitch data in real-time.
*   **Gyroscope Calibration:** Calibrates the gyroscope on startup for more accurate readings.

## Hardware Requirements

*   Two ESP32-C3 development boards (e.g., XIAO ESP32-C3)
*   One MPU6050 accelerometer/gyroscope module
*   Jumper wires
*   A computer to run the Python visualization script

## Software Requirements

*   [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html) (Espressif IoT Development Framework)
*   Python 3
*   `pyserial` Python package (`pip install pyserial`)
*   `matplotlib` Python package (`pip install matplotlib`)

## Installation and Setup

### 1. Hardware Setup

*   **Left Device (Transmitter):**
    *   Connect the MPU6050 to the ESP32-C3 as follows:
        *   MPU6050 VCC -> ESP32-C3 3V3
        *   MPU6050 GND -> ESP32-C3 GND
        *   MPU6050 SCL -> ESP32-C3 GPIO7
        *   MPU6050 SDA -> ESP32-C3 GPIO6
*   **Right Device (Receiver):**
    *   No external hardware is required for the receiver ESP32.

### 2. Firmware Setup

1.  **Clone the repository:**
    ```bash
    git clone <repository-url>
    ```
2.  **Configure the project:**
    *   Open the project in your preferred ESP-IDF environment.
    *   The code automatically determines the device's role (left or right) based on its MAC address. You will need to identify the MAC addresses of your two ESP32 devices and update the `mac_right` and `mac_left` arrays in `main/main.cpp`.
3.  **Build and Flash:**
    *   Build and flash the firmware onto both ESP32 devices using the ESP-IDF tools.

## Usage

1.  **Power on both ESP32 devices.**
2.  **Connect the receiver ESP32 to your computer via USB.**
3.  **Run the Python plotter script:**
    ```bash
    python main/plotter.py <your-serial-port>
    ```
    (Replace `<your-serial-port>` with the correct serial port for your receiver ESP32, e.g., `COM3` on Windows or `/dev/ttyUSB0` on Linux).
4.  A plot will appear, showing the real-time roll and pitch data from the MPU6050.

## Future Improvements

*   **Use LittleFS:** Replace NVS with LittleFS for file storage.
*   **Data Logging:** Save the sensor data to a file on the ESP32 or the computer for later analysis.
*   **3D Visualization:** Create a 3D visualization of the barbell's movement.
*   **Web Interface:** Create a web interface to display the data, eliminating the need for the Python script.