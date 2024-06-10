# ~/catkin_ws/src/gopigo3_ultrasound/src/ultrasonic_sensor.py
import RPi.GPIO as GPIO
import time

def setup_gpio(trigger_pin, echo_pin):
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(trigger_pin, GPIO.OUT)
    GPIO.setup(echo_pin, GPIO.IN)

def read_distance(trigger_pin, echo_pin):
    GPIO.output(trigger_pin, True)
    time.sleep(0.00001)
    GPIO.output(trigger_pin, False)

    start_time = time.time()
    stop_time = time.time()

    while GPIO.input(echo_pin) == 0:
        start_time = time.time()

    while GPIO.input(echo_pin) == 1:
        stop_time = time.time()

    time_elapsed = stop_time - start_time
    distance = (time_elapsed * 34300) / 2

    return distance