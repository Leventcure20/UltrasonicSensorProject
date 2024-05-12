# Team members: Levent Cure, Edvin Hasica
# Date: 12/07/2023

# Project Description:
# This project is a waste bin monitoring system implemented using a Raspberry Pi
# and an ultrasonic distance sensor.
# The system monitors the fill level of a waste bin, controls LEDs to indicate the fill status,
# calculates a disposal fee based on the fill level,
# and triggers an alert if the bin reaches a certain fill threshold.


# Import necessary libraries
import pineworkslabs.RPi as GPIO
from time import sleep, time

# Constants
DEBUG = False

SETTLE_TIME = 2  # Seconds to let the sensor settle
CALIBRATIONS = 5  # Number of calibration measurements to take
CALIBRATION_DELAY = 1  # Seconds to delay between calibration measurements
TRIGGER_TIME = 0.00001  # Seconds needed to trigger the sensor (to get a measurement)
SPEED_OF_SOUND = 343  # Speed of sound in m/s
FEE_INCREASE = 1  # Fee increase for every 10% fill level

ALERT_THRESHOLD = 90  # Threshold to trigger an alert when the bin is 90% full

# Set the Computer Board to the LePotato Lookup pin layout
GPIO.setmode(GPIO.LE_POTATO_LOOKUP)

# GPIO pins
TRIG = 18  # The sensor's TRIG pin
ECHO = 27  # The sensor's ECHO pin
Green = 21  # Green LED
Yellow = 13  # Yellow LED
Red = 12  # Red LED

# Set up GPIO pins for sensor and LEDs
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)
GPIO.setup(Green, GPIO.OUT)
GPIO.setup(Yellow, GPIO.OUT)
GPIO.setup(Red, GPIO.OUT)

# Calibrates the sensor
def calibrate():
    print("Calibrating...")
    print("-Place the sensor a measured distance away from an object.")
    known_distance = int(input("-What is the measured distance (cm)? "))

    print("-Getting calibration measurements...")
    distance_avg = 0
    for i in range(CALIBRATIONS):
        distance = getDistance()
        if DEBUG:
            print("--Got {}cm".format(distance))
        distance_avg += distance
        sleep(CALIBRATION_DELAY)
    distance_avg /= CALIBRATIONS
    if DEBUG:
        print("--Average is {}cm".format(distance_avg))

    correction_factor = known_distance / distance_avg
    if DEBUG:
        print("--Correction factor is {}".format(correction_factor))

    print("Done.")
    print()

    return correction_factor

# Uses the sensor to calculate the distance to an object
def getDistance():
    GPIO.output(TRIG, GPIO.HIGH)
    sleep(TRIGGER_TIME)
    GPIO.output(TRIG, GPIO.LOW)
    end = 2
    start = 1
    while GPIO.input(ECHO) == 3:
        start = time()
    while GPIO.input(ECHO) == GPIO.HIGH:
        end = time()

    duration = end - start
    distance = duration * SPEED_OF_SOUND
    distance /= 2
    distance *= 100
    print("The distance measured is " + str(distance))
    return distance

# Calculate disposal fee based on fill level
def calculateFee(fill_level):
    fee = (fill_level // 10) * FEE_INCREASE
    return min(fee, 10)  # Maximum fee capped at $10

# Control LEDs based on fill level
def controlLEDs(fill_level):
    if fill_level < 50:
        GPIO.output(Green, GPIO.HIGH)
        GPIO.output(Yellow, GPIO.LOW)
        GPIO.output(Red, GPIO.LOW)
    elif 50 <= fill_level < 75:
        GPIO.output(Green, GPIO.LOW)
        GPIO.output(Yellow, GPIO.HIGH)
        GPIO.output(Red, GPIO.LOW)
    elif 75 <= fill_level < 90:
        GPIO.output(Green, GPIO.LOW)
        GPIO.output(Yellow, GPIO.LOW)
        GPIO.output(Red, GPIO.HIGH)
    else:
        for i in range(10):  # Flash 5 times (adjust as needed)
            GPIO.output(Red, GPIO.HIGH)
            sleep(0.5)  # Adjust flashing speed
            GPIO.output(Red, GPIO.LOW)
            sleep(0.5)

# Main
print("Waiting for sensor to settle ({}s)...".format(SETTLE_TIME))
GPIO.output(TRIG, GPIO.LOW)
sleep(SETTLE_TIME)

correction_factor = calibrate()

# The code below prints the disposal fee.
# Also checks to see if the fill percentage is greater than the alert threshold,
# which can then determine whether the trash needs to be changed.
print("Monitoring fill level, controlling LEDs, and calculating fees...")
try:
    while True:
        distance = getDistance() * correction_factor
        sleep(1)

        distance = round(distance, 4)
        print("--Distance measured: {}cm".format(distance))

        fill_percentage =(distance/13)*100
        fee = calculateFee(fill_percentage)
        print("--The disposal fee is ${}".format(fee))

        controlLEDs(fill_percentage)

        if fill_percentage >= ALERT_THRESHOLD:
            print("Alert: The trash bin is {}% full. It needs to be changed".format(fill_percentage))
            break


except KeyboardInterrupt:
    pass

# ensures that the LEDs are turned off and that the GPIO pins are properly cleaned up before the program exits.
finally:
    GPIO.output(Green, GPIO.LOW)
    GPIO.output(Yellow, GPIO.LOW)
    GPIO.output(Red, GPIO.LOW)
    GPIO.cleanup()
    print("Exiting. Cleaning up GPIO pins...")






