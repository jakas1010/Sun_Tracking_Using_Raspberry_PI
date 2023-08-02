# Sun_Tracking_Using_Raspberry_PI

# Solar Tracking System for Antennas and Solar Panels

This Python script controls a solar tracking system designed to optimize the orientation of antennas or solar panels toward the sun. Originally developed for antenna tracking, it can also be used for solar panels and related tasks. The system uses a Raspberry Pi and a motor driver to dynamically adjust azimuth and elevation angles, enhancing energy efficiency and data collection.

## Table of Contents

- [Introduction](#introduction)
- [Prerequisites](#prerequisites)
- [Hardware Setup](#hardware-setup)
- [Usage](#usage)
- [Configuration](#configuration)
- [Acknowledgments](#acknowledgments)
- [License](#license)

## Introduction

This solar tracking system was originally designed for antenna applications, optimizing their orientation to improve signal reception. However, it can also be adapted for solar panels and other related tasks, enhancing energy efficiency and data collection capabilities. The system calculates azimuth and elevation angles based on time and location and adjusts the equipment accordingly.

## Prerequisites

- Raspberry Pi with Raspbian OS installed
- Motor driver compatible with Raspberry Pi (e.g., A4988)
- Stepper motors for azimuth and elevation control
- Magnetometer (LIS3MDL) for angle measurement
- Python 3.x installed

## Hardware Setup

Connect the pins on the Raspberry Pi to the motor driver and sensors as follows:

### Motor Driver Pins

- Connect `AZIMUTH_STEP_PIN` to the step input of the azimuth motor.
- Connect `AZIMUTH_DIR_PIN` to the direction input of the azimuth motor.
- Connect `ELEVATION_STEP_PIN` to the step input of the elevation motor.
- Connect `ELEVATION_DIR_PIN` to the direction input of the elevation motor.

### Magnetometer (LIS3MDL)

- Connect the I2C pins (SDA, SCL) of the magnetometer to the respective pins on the Raspberry Pi.

## Usage

1. Clone this repository to your Raspberry Pi.
2. Install the required Python packages: `numpy`, `smbus`, `matplotlib`, and `RPi.GPIO`. You can install them using `pip`:
   ```sh
   pip install numpy smbus matplotlib RPi.GPIO
