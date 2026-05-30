
# Dual Robotiq Gripper interface via ESP-NOW Wireless Bridge

This repository contains the firmware which is required for a custom wireless bridge system that controls two physical Robotiq grippers using ROS 2 (`ros2_control`).

Instead of hardwiring the grippers to the host PC via long RS485 USB cables, this project uses an **M5Stack AtomS3U (Master)** plugged into the PC to wirelessly route Modbus RTU commands to **two separate ESP32s (Slaves)** via the low-latency ESP-NOW protocol.


## System requirement

- Master: 1x [AtomS3U M5 stack](https://docs.m5stack.com/en/core/AtomS3U)
- Slaves: 2x ESP32S3 PCB [here](https://github.com/hardware-designs-iai/tiago_robotiq_adapter/tree/main/pcbs)
- Actuators: 2x Robotiq Grippers
- [ESP-IDF v6.0.1](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/index.html)


## Getting Started

The specific instructions for building and flashing the firmware are located in their respective directories:

- Navigate to the `AtomS3-M5_stack` directory for AtomS3U (Master) flashing instructions.

- Navigate to the `gripper` directory for the Gripper PCB (Slave) flashing instructions.