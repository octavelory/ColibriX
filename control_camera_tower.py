import pygame
from gpiozero import Servo
from time import sleep

# --- CONFIGURATION ---
# Broches GPIO pour les servos
SERVO1_PIN = 12
SERVO2_PIN = 13

# Axes de la manette
AXIS_SERVO_1 = 0
AXIS_SERVO_2 = 1

# Zone morte
DEADZONE = 0.08

# --- NOUVEAU PARAMÈTRE DE VITESSE ---
# Facteur de lissage. Plus cette valeur est PETITE, plus le mouvement sera LENT.
# - 0.1  : Assez rapide
# - 0.05 : Vitesse moyenne, bon compromis
# - 0.02 : Assez lent et fluide
# - 1.0  : Mouvement instantané
SMOOTHING_FACTOR = 0.07

# --- CONFIGURATION AVANCÉE DES SERVOS ---
MIN_PULSE = 0.5 / 1000
MAX_PULSE = 2.5 / 1000

# --- FONCTION D'AIDE ---
def clamp(value, min_val=-1.0, max_val=1.0):
    return max(min_val, min(value, max_val))

# --- INITIALISATION DES SERVOS ---
try:
    servo1 = Servo(SERVO1_PIN, min_pulse_width=MIN_PULSE, max_pulse_width=MAX_PULSE)
    servo2 = Servo(SERVO2_PIN, min_pulse_width = 1 / 1000, max_pulse_width = 2 / 1000)
except Exception as e:
    print(f"ERREUR lors de l'initialisation des servos : {e}")
    exit()

# --- VARIABLES D'ÉTAT POUR LE LISSAGE ---
# Position cible (donnée par le joystick)
target_servo1_pos = 0.0
target_servo2_pos = 0.0

# Position actuelle (celle que l'on va faire évoluer doucement)
current_servo1_pos = 0.0
current_servo2_pos = 0.0

# Positionner les servos au centre au démarrage
servo1.value = current_servo1_pos
servo2.value = current_servo2_pos
print("Servos initialisés pour un mouvement lent et fluide.")

# --- INITIALISATION DE PYGAME ET DE LA MANETTE ---
pygame.init()
if pygame.joystick.get_count() == 0:
    print("ERREUR: Aucune manette n'est connectée !")
    exit()
joystick = pygame.joystick.Joystick(0)
joystick.init()
print(f"Manette '{joystick.get_name()}' détectée. Vitesse réglée avec SMOOTHING_FACTOR={SMOOTHING_FACTOR}")
print("Contrôlez les servos avec les sticks. Appuyez sur Ctrl+C pour arrêter.")

try:
    while True:
        # 1. Mettre à jour la POSITION CIBLE avec les événements du joystick
        for event in pygame.event.get():
            if event.type == pygame.JOYAXISMOTION:
                if event.axis == AXIS_SERVO_1:
                    value = clamp(- event.value)
                    target_servo1_pos = value if abs(value) > DEADZONE else 0.0
                
                elif event.axis == AXIS_SERVO_2:
                    value = clamp(- event.value)
                    target_servo2_pos = value if abs(value) > DEADZONE else 0.0

        # 2. Calculer le prochain petit pas vers la cible (INTERPOLATION)
        # La formule est : nouvelle_pos = ancienne_pos + (cible - ancienne_pos) * facteur
        current_servo1_pos += (target_servo1_pos - current_servo1_pos) * SMOOTHING_FACTOR
        current_servo2_pos += (target_servo2_pos - current_servo2_pos) * SMOOTHING_FACTOR

        # 3. Appliquer la nouvelle position (lissée) aux servos
        servo1.value = current_servo1_pos
        servo2.value = current_servo2_pos
        
        # Petite pause pour contrôler la vitesse de la boucle
        sleep(0.01)

except KeyboardInterrupt:
    print("\nArrêt du programme.")

finally:
    print("Libération des broches GPIO et arrêt de pygame.")
    if 'servo1' in locals():
        servo1.detach()
    if 'servo2' in locals():
        servo2.detach()
    pygame.quit()
