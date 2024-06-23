# GoPiGo3 Differential Drive and Ultrasonic Sensor ROS Package
## Introduction
This ROS package provides a differential drive controller for the GoPiGo3 robot along with support for ultrasonic sensor data acquisition. The package includes launch files and scripts to control the robot's movement and gather distance measurements using ROS.

## Installation
1. Clone this repository into your catkin workspace
```
cd ~/catkin_ws/src
git clone https://github.com/KhertishL/Gopigo_group202_ROBO.git
```

3. Build the package:
   
```
cd ~/catkin_ws
catkin_make
```

3.Source the setup file:

```
source devel/setup.bash
```

4.Launching Differential Drive Controller
To launch the differential drive controller, use roslaunch:
`roslaunch gopigo3_bringup driver_differential.launch`

5.Running main code
`rosrun gopigo3_ultrasound drive2.py`

## Notes
Ensure the GoPiGo3 robot is properly connected and powered on before running the launch files and scripts.
Adjust any necessary parameters in driver_differential.launch or drive2.py to suit your specific setup or requirements.

## Code drive2.py explanations

#### 1. Measuring the distance
```
def measure_distance(trig_pin, echo_pin):
    GPIO.output(trig_pin, True)
    time.sleep(0.00001)
    GPIO.output(trig_pin, False)

    pulse_start = time.time()
    pulse_end = time.time()

    while GPIO.input(echo_pin) == 0:
        pulse_start = time.time()

    while GPIO.input(echo_pin) == 1:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150  # Speed of sound in cm/s
    return round(distance, 2)
```
This section triggers the ultrasonic sensor to send out an ultrasonic pulse.
GPIO.output(trig_pin, True): Sets the GPIO pin connected to the sensor's trigger (trig_pin) to HIGH, initiating the ultrasonic pulse.
time.sleep(0.00001): Pauses execution for 10 microseconds (0.00001 seconds). This short delay is necessary to ensure the trigger pulse is detected by the sensor.
GPIO.output(trig_pin, False): Sets the trigger pin back to LOW, ending the ultrasonic pulse.

pulse_start = time.time(): Records the current time when the echo pin (echo_pin) first becomes HIGH (indicating the start of the echo pulse).
while GPIO.input(echo_pin) == 0:: Loops and updates pulse_start until the echo pin goes HIGH.
pulse_end = time.time(): Records the time when the echo pin returns LOW (indicating the end of the echo pulse).
while GPIO.input(echo_pin) == 1:: Loops and updates pulse_end until the echo pin goes LOW.

pulse_duration = pulse_end - pulse_start: Calculates the duration of the echo pulse, which is the time taken for the ultrasonic pulse to travel to an object and back.
distance = pulse_duration * 17150: Calculates the distance based on the formula: distance = (time * speed_of_sound) / 2. Since the ultrasonic pulse travels to the object and back, we divide by 2. The speed of sound is approximately 343 meters per second or 34300 centimeters per second. 

#### 2. Checking obstacle
```
def check_obstacles(trig_pins, echo_pins, threshold):
    distances = [measure_distance(trig, echo) for trig, echo in zip(trig_pins, echo_pins)]
    obstacle_detected = any(distance < threshold for distance in distances)
    return obstacle_detected, distances
```
The check_obstacles function is designed to determine if there are any obstacles detected by ultrasonic sensors connected to the GPIO pins of a Raspberry Pi. It uses the measure_distance function to get distance readings from each sensor and then compares these readings against a specified threshold to decide if an obstacle is present.

zip(trig_pins, echo_pins): Combines trig_pins and echo_pins into pairs of (trig, echo) for each sensor.
[measure_distance(trig, echo) for trig, echo in zip(trig_pins, echo_pins)]: Uses a list comprehension to iterate over each (trig, echo) pair, calling measure_distance(trig, echo) to measure the distance for each sensor.

any(distance < threshold for distance in distances): Uses the any() function, which returns True if any element in the iterable (distances in this case) satisfies the condition (distance < threshold).


#### 3.Controlling the robot
```
def control_robot(pub, forward):
    twist = Twist()
    if forward:
        twist.linear.x = 0.2  # Move forward
    else:
        twist.linear.x = 0.0
    twist.angular.z = 0.0
    pub.publish(twist)
```
control_robot(pub, forward): Publishes Twist messages to control robot movement (forward or stop).

```
def turn(angle, pub):
    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = 0.5 if angle > 0 else -0.5  # Rotation speed based on angle

    turn_duration = abs(angle) / 90.0  # Adjust for precise rotation
    end_time = rospy.Time.now() + rospy.Duration(turn_duration)

    while rospy.Time.now() < end_time:
        pub.publish(twist)
        rospy.sleep(0.1)

    twist.angular.z = 0.0
    pub.publish(twist)
```
turn(angle, pub): Rotates the robot by publishing angular Twist commands.
abs(angle): Takes the absolute value of the angle to ensure a positive duration, regardless of the direction of rotation.
/ 90.0: Divides by 90.0 to scale the duration based on the rotation angle. This assumes that the robot takes 90 units for the rotation

#### 4. Main loop main function
```
if distances[1] > 200:  # Arbitrary distance condition
                stop_robot(pub)
                break

            if obstacle_detected:
                distance_left = distances[0]
                distance_right = distances[2]

                if distance_left > distance_right:
                    turn(90, pub)  # Turn left
                else:
                    turn(-90, pub)  # Turn right
            else:
                control_robot(pub, True)  # Move forward if no obstacles
```
The is statement verify the distances to see in which direction there is an obstacle. distance[0] and distance[2] being left and right sensor.

