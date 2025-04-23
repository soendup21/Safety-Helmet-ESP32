# Safety Helmet ESP32

## Overview
This project implements a smart safety helmet using an ESP32 microcontroller. The helmet is equipped with sensors to detect accidents, monitor the wearer's health, and send notifications in case of emergencies. It uses Wi-Fi connectivity to send alerts and GPS to provide location tracking.

## Features
- Accident detection using MPU-6050 accelerometer and gyroscope.
- GPS location tracking.
- Heart rate monitoring.
- Notification alerts via LINE Notify.
- Helmet wear detection.

## Hardware Requirements
- ESP32 microcontroller.
- MPU-6050 accelerometer and gyroscope module.
- GPS module.
- Heart rate sensor.
- Buzzer.
- LEDs (Green and Yellow).
- Push button.
- Helmet with a detector pin.

## Software Requirements
- Arduino IDE.
- Required libraries:
  - `Wire.h`
  - `TridentTD_LineNotify.h`
  - `SoftwareSerial.h`
  - `TinyGPS++.h`

## Setup Instructions
1. Clone this repository to your local machine.
2. Open the Arduino IDE and install the required libraries.
3. Connect the hardware components as per the pin definitions in the code.
4. Update the following constants in the code with your Wi-Fi and LINE Notify credentials:
   ```c
   #define SSID "Your_WiFi_SSID"
   #define PASSWORD "Your_WiFi_Password"
   #define LINE_TOKEN "Your_LINE_Notify_Token"
   ```
5. Upload the code to the ESP32 using the Arduino IDE.
6. Power on the ESP32 and ensure all components are functioning correctly.

## Usage
- Wear the helmet to activate the detector pin.
- In case of an accident, the system will:
  1. Detect the crash using the MPU-6050 sensor.
  2. Send a notification with the GPS location to the configured LINE Notify account.
  3. Monitor the wearer's heart rate and send updates.
- Press the button to abort the notification in case of a false alarm.

## Pin Configuration
| Component       | Pin  |
|-----------------|------|
| PIR Sensor      | 16   |
| GPS RX          | 39   |
| GPS TX          | 36   |
| Buzzer          | 15   |
| Button          | 27   |
| Detector Pin    | 13   |
| LED (Yellow)    | 14   |
| LED (Green)     | 12   |

## License
This project is licensed under the MIT License. See the LICENSE file for details.