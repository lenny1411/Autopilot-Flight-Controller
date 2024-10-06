The goal of this project is to build a flight controller with GPS navigation based on ESP32-S3. The embedded software is designed to be as portable as possible.

## Project status:
- Low-level interfaces (Done)
- Devices interfaces (Done, note: Barometer interface is not implemented)
- Modules (In progress)
- Telemetry (In progress)

## Technologies used:
- Arduino Framework + ESP-IDF
- PlatformIO + (Clion IDE or VS Code)
- ESP32-S3
- Arduino libraries rewritten for portability to control devices like IMUs, barometers, receivers, etc...
- Pub/Sub system for communication between task modules

## Hardware:
![20240211_153857](https://github.com/lenny1411/Autopilot-Flight-Controller/assets/105748537/35af56b6-37de-48b7-aed2-a2a18e67e7e1)

Components:
- ESP32-S3
- IMU 6-Dof BMI088 Gyro+Accel
- Transmitter Flysky FS-I6X + Receiver Flysky FS-IA6B (Ibus mode)
- ZED-F9P RTK GNSS From ArduSimple
- BMM150 (magnetometer)
- MS5611 (barometer)
- 4 or 6 ESC 30A 3s/2s + BEC 5V
- 3.3V Regulator
- 3s 8000 mAh Lipo battery XT60
## Hardware block diagram:
Coming soon.

## Monitoring and telemetry:
[Android App](https://github.com/Embedded-MUTEX-1/DroneMonitoringApp)

## Flight in position hold and altitude hold mode (V1) :
https://github.com/user-attachments/assets/1a04a159-e1b1-4e9f-b5fd-9d827e6439e0

## Software architecture :
![Autopilot](https://github.com/Embedded-MUTEX-1/Autopilot-Flight-Controller/assets/105748537/b565809f-8e8a-446c-8b6c-04ee01bfed2f)
