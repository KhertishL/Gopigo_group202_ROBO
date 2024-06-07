import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
#GPIO.setwarnings(False)
# Broches pour les trois capteurs ultrasoniques
TRIG = [17, 26, 19]
ECHO = [21, 20, 16]

# Broche pour la LED
LED = 27
 
for trig_pin in TRIG: GPIO.setup(trig_pin, GPIO.OUT)

for echo_pin in ECHO:
    GPIO.setup(echo_pin, GPIO.IN)

GPIO.setup(LED, GPIO.OUT)

# Attente pour que les capteurs se stabilisent
for trig_pin in TRIG:
    GPIO.output(trig_pin, False)
print("Waiting for sensors to settle")
time.sleep(2)

try:
    while True:
        obstacle_detected = False

        # Mesure de la distance pour chaque capteur
        for i in range(len(TRIG)):
            GPIO.output(TRIG[i], True)
            time.sleep(0.00001)
            GPIO.output(TRIG[i], False)

            while GPIO.input(ECHO[i]) == 0:
                pulse_start = time.time()

            while GPIO.input(ECHO[i]) == 1:
                pulse_end = time.time()

            pulse_duration = pulse_end - pulse_start
            distance = pulse_duration * 17150
            distance = round(distance, 2)

            print(f"Distance {i+1}: {distance} cm")

            # Activation de l'indicateur d'obstacle si la distance est inférieure ou égale à un seuil
            threshold = 10  # Ajustez ce seuil si nécessaire
            if distance <= threshold:
                obstacle_detected = True

        # Activation de la LED si un obstacle est détecté par au moins un capteur
        if obstacle_detected:
            GPIO.output(LED, GPIO.HIGH)
        else:
            GPIO.output(LED, GPIO.LOW)

        time.sleep(1)

except KeyboardInterrupt:
    GPIO.cleanup()
