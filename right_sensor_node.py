#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# Configuration des broches GPIO et initialisation des capteurs et LED
def setup_gpio(trig_pins, echo_pins, led_pin):
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)  # Désactiver les avertissements GPIO

    for trig_pin in trig_pins:
        GPIO.setup(trig_pin, GPIO.OUT)
        GPIO.output(trig_pin, False)  # Stabilisation initiale

    for echo_pin in echo_pins:
        GPIO.setup(echo_pin, GPIO.IN)

    GPIO.setup(led_pin, GPIO.OUT)

    print("Waiting for sensors to settle")
    time.sleep(2)

# Mesure de la distance avec un capteur ultrasonique
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
    distance = pulse_duration * 17150
    return round(distance, 2)

# Vérification de la présence d'obstacles
def check_obstacles(trig_pins, echo_pins, threshold):
    distances = [measure_distance(trig, echo) for trig, echo in zip(trig_pins, echo_pins)]
    obstacle_detected = any(distance < threshold for distance in distances)
    return obstacle_detected, distances

# Contrôle du robot avec ROS
def control_robot(pub, forward):
    twist = Twist()
    if forward:
        twist.linear.x = 0.5  # Avancer
    else:
        twist.linear.x = 0.0
    twist.angular.z = 0.0
    pub.publish(twist)

# Fonction de rotation du robot à gauche
def turn_left(angle, pub):
    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = 0.5  # Vitesse de rotation

    turn_duration = angle / 45.0  # Ajustez ce facteur pour une rotation précise
    end_time = rospy.Time.now() + rospy.Duration(turn_duration)

    while rospy.Time.now() < end_time:
        pub.publish(twist)
        rospy.sleep(0.1)

    twist.angular.z = 0.0
    pub.publish(twist)

# Fonction de rotation du robot à droite
def turn_right(angle, pub):
    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = -0.5  # Vitesse de rotation

    turn_duration = angle / 45.0  # Ajustez ce facteur pour une rotation précise
    end_time = rospy.Time.now() + rospy.Duration(turn_duration)

    while rospy.Time.now() < end_time:
        pub.publish(twist)
        rospy.sleep(0.1)

    twist.angular.z = 0.0
    pub.publish(twist)

# Fonction de choix de direction basée sur les données des capteurs
def choose_direction(distances, threshold, pub):
    if distances[2] < threshold:  # Obstacle à droite
        turn_left(90, pub)  # Tourner à gauche de 90 degrés
        print("Obstacle détecté à droite, tournant à gauche de 90 degrés")
    elif distances[1] < threshold:  # Obstacle à gauche
        turn_right(90, pub)  # Tourner à droite de 90 degrés
        print("Obstacle détecté à gauche, tournant à droite de 90 degrés")
    else:
        control_robot(pub, True)  # Avancer si aucun obstacle n'est détecté

# Fonction principale
if __name__ == '__main__':
    rospy.init_node('robot_controller')

    trig_milieu = 26
    trig_gauche = 17
    trig_droite = 19
    echo_milieu = 20
    echo_gauche = 21
    echo_droite = 16
    led_pin = 18
    threshold = 5  # Distance seuil en cm

    trig_pins = [trig_milieu, trig_gauche, trig_droite]
    echo_pins = [echo_milieu, echo_gauche, echo_droite]

    setup_gpio(trig_pins, echo_pins, led_pin)

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    rate = rospy.Rate(10)  # 10 Hz
    try:
        while not rospy.is_shutdown():
            obstacle_detected, distances = check_obstacles(trig_pins, echo_pins, threshold)
            GPIO.output(led_pin, GPIO.HIGH if obstacle_detected else GPIO.LOW)

            choose_direction(distances, threshold, pub)

            # Afficher les distances mesurées et les mouvements du robot
            print(f"Distances: Avant: {distances[0]} cm, Gauche: {distances[1]} cm, Droite: {distances[2]} cm")
            print(f"LED {'allumée' if obstacle_detected else 'éteinte'}")

            rate.sleep()
    except rospy.ROSInterruptException:
        pass
    finally:
        GPIO.cleanup()