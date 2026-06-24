# ESP32 GSM Car Alarm

Autonomous GSM/GPS car alarm system for Moskvich 412 based on ESP32 and the LILYGO T-SIM7600G-H module.

## Description

This project is an autonomous GSM/GPS car alarm system designed for a Moskvich 412. It is based on an ESP32 board with the LILYGO T-SIM7600G-H module.

The system is installed inside the car and detects shocks, vibration, door/hood opening and other security events. When an alarm is triggered, the device can activate a siren and send SMS notifications to the owner.

The alarm is already installed in the car and is being tested in real conditions.

## Features

* Shock and vibration detection using MPU6050;
* reed switch input for door/hood detection;
* siren control through a MOSFET;
* status LED indicator;
* SMS notifications using SIM7600;
* GPS location support;
* adjustable sensitivity and runtime parameters;
* OTA/web configuration support;
* 3D-printable mounts for the sensor and LED indicator.

## Hardware

Main components:

* LILYGO T-SIM7600G-H / ESP32-WROVER;
* MPU6050 accelerometer;
* SIM7600 GSM/GPS modem;
* reed switch;
* siren;
* MOSFET for siren control;
* external battery;
* 3D-printed enclosure and mounting parts.

## Pinout

| Function       |   GPIO |
| -------------- | -----: |
| I2C SDA        | GPIO21 |
| I2C SCL        | GPIO22 |
| MPU6050 INT    | GPIO14 |
| Reed switch    | GPIO13 |
| Siren          | GPIO32 |
| Status LED     | GPIO19 |
| BAT ADC        | GPIO35 |
| SIM7600 TX     | GPIO27 |
| SIM7600 RX     | GPIO26 |
| SIM7600 PWRKEY |  GPIO4 |
| SIM7600 DTR    | GPIO25 |

## Project Structure

```text
.
├── include/                 # Header files
├── src/                     # Main firmware source code
├── platformio.ini           # PlatformIO build configuration
├── partitions_ota.csv       # OTA partition table
├── README.md
├── moskvich_alarm_3d.html   # 3D project visualization
├── mpu6050_car_body_mount.stl
├── led_window_holder_5mm.stl
└── images/                  # Wiring diagrams, renders and reference images
```

## Build and Upload

The project is built using PlatformIO.

Build:

```bash
pio run
```

Upload firmware:

```bash
pio run -t upload
```

Serial monitor:

```bash
pio device monitor
```

## 3D Models

The repository includes STL files:

* `mpu6050_car_body_mount.stl` — MPU6050 mount for attaching the sensor to the car body;
* `led_window_holder_5mm.stl` — 5 mm LED holder for placing the status indicator near the window.

## Images and Diagrams

The repository contains wiring diagrams, board reference images and project visualizations.

Recommended structure:

```text
images/
├── wiring-diagram.png
├── 3d-preview.png
└── lilygo-pinout.jpg
```

## Project Status

The project is under active development and real-world testing.

Current status:

* installed in the car;
* basic alarm logic is working;
* SMS notifications are working;
* MPU6050 sensitivity tuning is in progress;
* enclosure, mounting parts and power circuit are being improved.
