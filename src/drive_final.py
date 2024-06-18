#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# Configuration des broches GPIO et initialisation des capteurs et LED
def setup_gpio(trig_pins, echo_pins):
    print("setup function test")
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)  # Désactiver les avertissements GPIO

    for trig_pin in trig_pins:
        GPIO.setup(trig_pin, GPIO.OUT)
        GPIO.output(trig_pin, False)  # Stabilisation initiale

    for echo_pin in echo_pins:
        GPIO.setup(echo_pin, GPIO.IN)

    print("Waiting for sensors to settle")
    time.sleep(2)

# Mesure de la distance avec un capteur ultrasonique
def measure_distance(trig_pin, echo_pin):
    print("Measure distance test")
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
    distance = pulse_duration * 17150
    return round(distance, 2)

# Vérification de la présence d'obstacles
def check_obstacles(trig_pins, echo_pins, threshold):
    print("check obstacle test")
    distances = [measure_distance(trig, echo) for trig, echo in zip(trig_pins, echo_pins)]
    obstacle_detected = any(distance < threshold for distance in distances)
    return obstacle_detected, distances

# Contrôle du robot avec ROS
def control_robot(pub, forward):
    print("control_robot test")
    twist = Twist()
    if forward:
        twist.linear.x = 0.1  # Avancer
    else:
        twist.linear.x = 0.0
    twist.angular.z = 0.0
    pub.publish(twist)

# Fonction de rotation du robot
def turn(angle, pub):
    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = 0.5 if angle > 0 else -0.5  # Vitesse de rotation en fonction de l'angle

    # Calcule la durée de rotation basée sur l'angle
    turn_duration = abs(angle) / 90.0  # Ajustez ce facteur pour une rotation précise
    end_time = rospy.Time.now() + rospy.Duration(turn_duration)

    while rospy.Time.now() < end_time:
        pub.publish(twist)
        rospy.sleep(0.1)

    twist.angular.z = 0.0
    pub.publish(twist)

# Fonction pour avancer après avoir tourné
def move_forward(pub, duration):
    twist = Twist()
    twist.linear.x = 0.2  # Vitesse d'avance
    twist.angular.z = 0.0
    end_time = rospy.Time.now() + rospy.Duration(duration)

    while rospy.Time.now() < end_time:
        pub.publish(twist)
        rospy.sleep(0.1)

    twist.linear.x = 0.0
    pub.publish(twist)

# Fonction pour arrêter le robot
def stop_robot(pub):
    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    pub.publish(twist)
    print("Robot stopped")

# Fonction principale
if __name__ == '__main__':
    rospy.init_node('robot_controller')

    trig_pins = [17, 26, 19]  # Pins trig des capteurs avant, gauche et droite
    echo_pins = [21, 20, 16]  # Pins echo des capteurs avant, gauche et droite
    threshold = 10  # Distance seuil en cm

    setup_gpio(trig_pins, echo_pins)

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    rate = rospy.Rate(10)  # 10 Hz
    try:
        while not rospy.is_shutdown():
            obstacle_detected, distances = check_obstacles(trig_pins, echo_pins, threshold)
            print(obstacle_detected)

            # Check if left distance is greater than 175 cm
            if distances[1] > 175:
                stop_robot(pub)
                print(f"Left distance is {distances[1]} cm, stopping robot")
                break

            if obstacle_detected:
                # Get distances for left and right sensors
                distance_left = distances[0]
                distance_right = distances[2]
                
                # Determine the direction to turn
                if distance_left > distance_right:
                    turn(60, pub)  # Turn left by 45 degrees
                    print("Turning left by 45 degrees")
                else:
                    turn(-60, pub)  # Turn right by 45 degrees
                    print("Turning right by 45 degrees")
                
                # Move forward a bit after turning
                move_forward(pub, 1)  # Move forward for 1 second
            else:
                control_robot(pub, True)  # Avancer si aucun obstacle n'est détecté

            # Afficher les distances mesurées et les mouvements du robot
            print(f"Distances: Gauche: {distances[0]} cm, Avant: {distances[1]} cm, Droite: {distances[2]} cm")

            rate.sleep()
    except rospy.ROSInterruptException:
        pass
    finally:
        GPIO.cleanup()
