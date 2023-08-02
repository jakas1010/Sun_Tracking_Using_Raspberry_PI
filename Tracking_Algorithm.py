import numpy
import smbus   
import matplotlib.pyplot as plt 
import time
import math
import RPi.GPIO as GPIO
from datetime import datetime

# Pin Configuration for Raspberry Pi and Motor Driver
# Connect the following pins on Raspberry Pi to the respective pins on the motor driver.
# Update these pin numbers based on your actual connections.

# Motor driver pin numbers for azimuth motor
AZIMUTH_STEP_PIN = 13   # Connect this pin to the step input of azimuth motor
AZIMUTH_DIR_PIN = 18    # Connect this pin to the direction input of azimuth motor

# Motor driver pin numbers for elevation motor
ELEVATION_STEP_PIN = 2  # Connect this pin to the step input of elevation motor
ELEVATION_DIR_PIN = 3   # Connect this pin to the direction input of elevation motor


# Constants and parameters
startTime = 6.00
endTime = 18.00
minimumAngle = 7.5
months = [0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31]
longitude = 73.6698442
latitude = math.radians(24.6147902)
previousElevation = 0.0

I2C_BUS = 1
bus = smbus.SMBus(I2C_BUS)



# Magnetometer (LIS3MDL) I2C address and registers
MAGNETOMETER_ADDRESS = 0x1E
OUT_X_L = 0x28
OUT_X_H = 0x29
OUT_Y_L = 0x2A
OUT_Y_H = 0x2B
OUT_Z_L = 0x2C
OUT_Z_H = 0x2D

# Set up GPIO pins
def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(AZIMUTH_STEP_PIN, GPIO.OUT)
    GPIO.setup(AZIMUTH_DIR_PIN, GPIO.OUT)
    GPIO.setup(ELEVATION_STEP_PIN, GPIO.OUT)
    GPIO.setup(ELEVATION_DIR_PIN, GPIO.OUT)

# Get the current time and day number
def get_current_time_and_day():
    current_time = []
    current_datetime = datetime.now()
    day = 0.0
    
    local_time = current_datetime.hour + (current_datetime.minute / 60) + (current_datetime.second / 3600)
    for i in range(0, current_datetime.month - 1):
        day = months[i] + day
        time.sleep(0.01)
    day = day + current_datetime.day
    
    current_time.append(local_time)
    current_time.append(day)
    
    return current_time

# Calculate sun angles
def calculate_sun_angles(local_time, day, previous_elevation):
    angles = []
    
    local_solar_time_meridian = 15.0 * 5.5
    b = (day - 81) * 360.0 / 365.0
    b = math.radians(b)
    eot = 9.87 * math.sin(2.0 * b) - 7.52 * math.cos(b) - 1.5 * math.sin(b)
    tc = 4.0 * (longitude - local_solar_time_meridian) + eot 
    
    local_solar_time = local_time + (tc / 60.0)
    h = math.radians(15.0 * (local_solar_time - 12.0))
    dec = 23.45 * math.sin(b)
    dec = math.radians(dec)
    elevation = math.asin((math.sin(dec) * math.sin(latitude)) + (math.cos(dec) * math.cos(latitude) * math.cos(h)))
    azimuth = math.acos(((math.sin(dec) * math.cos(latitude)) - (math.cos(dec) * math.sin(latitude) * math.cos(h))) / math.cos(elevation))
    azimuth = math.degrees(azimuth)
    elevation = math.degrees(elevation)
    
    if previous_elevation > elevation:
        angles.append(360 - azimuth)
    else:
        angles.append(azimuth)
    angles.append(elevation)
    
    return angles

# Rotate stepper motor by a given angle
def rotate_stepper_motor(STEP_PIN, DIR_PIN, angle):
    steps_per_revolution = 200
    degrees_per_step = 360.0 / steps_per_revolution
    steps = int(angle * 64.25 / degrees_per_step)
    
    if angle >= 0:
        GPIO.output(DIR_PIN, GPIO.LOW)
    else:
        GPIO.output(DIR_PIN, GPIO.HIGH)
    
    delay = 0.0001
    timer_here = 0
    for i in range(abs(steps)):
        GPIO.output(STEP_PIN, GPIO.HIGH)
        time.sleep(delay)
        GPIO.output(STEP_PIN, GPIO.LOW)
        time.sleep(delay)
        timer_here += 2 * delay

# Read 16-bit registers from I2C
def read_i2c_register_16bit(address, register):
    high_byte = bus.read_byte_data(address, register + 1)
    low_byte = bus.read_byte_data(address, register)
    value = (high_byte << 8) | low_byte
    return value

# Read magnetometer angle
def read_magnetometer_angle():
    temp = []
    x = read_i2c_register_16bit(MAGNETOMETER_ADDRESS, OUT_X_L)
    y = read_i2c_register_16bit(MAGNETOMETER_ADDRESS, OUT_Y_L)
    z = read_i2c_register_16bit(MAGNETOMETER_ADDRESS, OUT_Z_L)
    angle_rad = math.atan2(y, x)
    angle_deg = math.degrees(angle_rad)
    if angle_deg < 0:
        angle_deg += 360
    temp.append(angle_deg)  # Azimuth angle
    angle_rad = math.atan2(z, x)
    angle_deg = math.degrees(angle_rad)
    if angle_deg < 0:
        angle_deg += 360
    temp.append(angle_deg)  # Elevation angle
    return temp

if __name__ == '__main__':
    setup()
    current_time_and_day = []
    angles = []
    actual_angle = [0, 0]
    
    while True:
        current_time_and_day = get_current_time_and_day()
        angles = calculate_sun_angles(current_time_and_day[0], current_time_and_day[1], actual_angle[1])
        
        if angles[1] >= minimumAngle and startTime <= current_time_and_day[0] <= endTime:
            rotate_stepper_motor(AZIMUTH_STEP_PIN, AZIMUTH_DIR_PIN, angles[0] - actual_angle[0])
            actual_angle[0] += angles[0] - actual_angle[0]
            if actual_angle[0] >= 360 or actual_angle[0] < 0:
                actual_angle[0] %= 360
            rotate_stepper_motor(ELEVATION_STEP_PIN, ELEVATION_DIR_PIN, angles[1] - actual_angle[1])
            actual_angle[1] += angles[1] - actual_angle[1]
            time.sleep(10)
        else:
            print("Currently in sleeping state")
            time.sleep(10)
