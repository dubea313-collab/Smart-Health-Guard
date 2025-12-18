# Non-Contact Temperature-Based Sanitization System

## Project Status
Working hardware prototype (proof-of-concept)

## Description
This project is a non-contact automatic sanitization system that measures
body temperature using an infrared temperature sensor and dispenses sanitizer
when a valid temperature is detected. The system was built and tested on a
breadboard to validate functionality.

## Features
- Non-contact temperature sensing
- Automatic sanitizer dispensing
- Relay-controlled pump
- External power isolation for motor safety

## Hardware Used
- Arduino Uno
- MLX90614 IR Temperature Sensor
- Relay Module
- DC Pump

## System Operation
1. IR sensor reads object temperature
2. Microcontroller evaluates temperature threshold
3. Relay activates pump if conditions are met
4. Sanitizer is dispensed automatically

## Limitations
- Breadboard-based prototype
- No enclosure
- Basic firmware logic

## Future Improvements
- PCB design
- LCD/OLED display
- IoT monitoring
- Enclosure and safety improvements

## Demo
See working prototype video in /media/demo_video_link.txt