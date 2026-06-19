# ALDL Digital Dash for 1995 Chevrolet C1500

A custom ESP32-S3 based digital dashboard for OBD1/ALDL-equipped GM vehicles.

This project runs on a 5" ESP32-S3 touchscreen display and reads live ALDL data directly from the vehicle ECM, presenting it in a configurable touch-driven dashboard. Each tile can be customized to display different engine and vehicle parameters, allowing the driver to build a dashboard that fits their needs.

<img width="3024" height="4032" alt="20260618_200750" src="https://github.com/user-attachments/assets/4a9354dc-765f-4647-a0d0-589e056e91f3" />

## Features

* 8192 baud GM ALDL support
* Designed and tested on a 1995 Chevrolet C1500 5.7L TBI
* 5" 800x480 capacitive touchscreen interface
* Customizable dashboard tiles
* Automatic F4/F5 frame polling based on selected dashboard fields
* Fast touch-based field selection
* ESP32-S3 platform with no web dependency
* Synthetic sensor support beyond ECM data

## Available Dashboard Data

Examples of supported fields include:

* RPM
* Vehicle Speed (MPH)
* Coolant Temperature
* Throttle Position Sensor (TPS)
* Battery Voltage
* Injector Pulse Width
* Block Learn Multiplier (BLM)
* Integrator (INT)
* Gear Position
* Fuel Pump Voltage
* Calculated MPG
* And many other ALDL parameters

## Cabin Temperature & Humidity Sensor

The dashboard now supports an onboard AHT temperature and humidity sensor.

The sensor is connected to a dedicated I²C bus using:

* GPIO10 (SDA)
* GPIO13 (SCL)

This allows environmental monitoring without interfering with the ALDL interface or touchscreen controller.

Displayed as:

```text
Cabin
80.6°F
43.5%
```

The sensor is only polled when the Cabin tile is actively selected on the dashboard, minimizing processor and bus activity.

<img width="2268" height="4032" alt="Picture_20260618081310" src="https://github.com/user-attachments/assets/3583f0c9-d179-4c97-aabf-8e6bf15c9156" />


## Hardware

### Display

* JC8048W550C / ESP32-8048S050C
* ESP32-S3 N16R8
* 800x480 RGB display
* GT911 capacitive touch controller

### ALDL Interface

* ESP32-S3 UART interface
* 74HC125 level/buffer interface
* 8192 baud GM ALDL support
<img width="3024" height="4032" alt="20260618_201219" src="https://github.com/user-attachments/assets/f8283794-205a-4337-9814-b205146ad80f" />
<img width="1536" height="1024" alt="ChatGPT Image Jun 4, 2026, 11_14_17 AM" src="https://github.com/user-attachments/assets/7c299b2d-4cbc-40c1-b5c0-4347c9fb50e5" />

Current wiring:

* RX: GPIO17
* TX: GPIO18
* RX Enable: GPIO11
* TX Output Enable: GPIO12

## Project Goals

The goal of this project is to provide a modern digital dashboard experience for OBD1 GM trucks while remaining inexpensive, open source, and easy to customize.

Future expansion may include:

* Additional environmental sensors
* Oil temperature monitoring
* Transmission temperature monitoring
* Data logging
* Warning and alert screens
* Expanded gauge views

## License

See repository license information.
