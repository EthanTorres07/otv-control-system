# ENES100 OTV Control System

Embedded C++ control software for an over terrain vehicle (OTV) for ENES100 at UMD.

## Overview
The OTV autonomously completes a seed-planting mission and traverses an obstacle course in a 4x2m arena. The software handles state-based navigation, driving DC motors for propulsion, ultrasonic obstacle detection, and wireless communication with an overhead vision system.

## Mission Details
The mission site consists of four pots containing either plantable substrate (orzo) or non-plantable substrate (rocks). The OTV must plant a seed (lima bean) into a pot and pick up 10g or more of rocks from the two front plants. The OTV must then use a machine learning algorithm to identify the two back pots as either plantable or non-plantable substrate and transmit the result to the overhead vision system.

## Machine Learning
The OTV uses a machine learning model to classify substrate in the two back pots as plantable (orzo) or non-plantable (rocks). The model was trained on ~160 hand-collected images and hosted on the ENES100 vision system. The ESP32-CAM captures and uploads images for cloud inference, with results transmitted back to the Arduino.


## Hardware
- Arduino Mega
- HC-SR04 ultrasonic sensor
- ESP32-CAM
- BTS7960 Motor Driver
- 4 driver motors with PWM control
- Actuator motors for bow system and rack and pinion system
- Mecanum wheels

## Software Architecture
- State machine: MISSION -> NAVIGATION
- Obstacle avoidance: ultrasonic ranging with lateral escape logic
- Position/heading feedback via Enes100 overhead vision system
- Proportional speed control (full/slow/crawl based on obstacle distance)

## Status
Hardware build and field demos in progress. Control software complete.

## Dependencies
- Enes100 Arduino library (enes100.umd.edu)

