"""
TP2 IA - Wargame
"""

from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Initialisation des capteurs de proximité
noms_capteurs = ['ps1', 'ps2', 'ps5', 'ps6']
capteurs = {}
for nom in noms_capteurs:
    capteurs[nom] = robot.getDevice(nom)
    capteurs[nom].enable(timestep)

# Initialisation des moteurs en mode vitesse
moteur_gauche = robot.getDevice('left wheel motor')
moteur_droit = robot.getDevice('right wheel motor')
moteur_gauche.setPosition(float('inf'))
moteur_droit.setPosition(float('inf'))

# Constantes
VMAX_ROUE = 6.28
VMAX_FLOU = 4.5
VAL_MAX = 4095.0
SEUIL_BRUIT = 0.03
GAIN_ANGLE = 0.15


def trapeze(x, a, b, c, d):
    if x < a or x > d:
        return 0.0
    if b <= x <= c:
        return 1.0
    if x < b:
        return (x-a) /(b-a)
    return (d-x) / (d-c)


def triangle(x, a, b, c):
    return trapeze(x, a, b, b, c)

# Fuzzy sets for normalised obstacle distance (0 = far, 1 = contact)

def loin(x):
    return trapeze(x, 0.0, 0.0, SEUIL_BRUIT, 0.17)

def moyen(x):
    return triangle(x, SEUIL_BRUIT, 0.17, 0.50)

def proche(x):
    return trapeze(x, 0.17, 0.50, 1.0, 1.0)

# Fuzzy sets for left-right imbalance (dist_gauche - dist_droite)
# Negative = right wall is closer; Positive = left wall is closer

def mur_plus_proche_droite(x):
    return trapeze(x, -1.0, -1.0, -0.25, -0.03)

def centre(x):
    return triangle(x, -0.15, 0.0, 0.15)

def mur_plus_proche_gauche(x):
    return trapeze(x, 0.03, 0.25, 1.0, 1.0)

# Output fuzzy sets: steering angle (degrees, > 0 = turn right)

def tourner_gauche(y):
    return trapeze(y, -30, -30, -15, 0)

def tout_droit(y):
    return triangle(y, -10, 0, 10)

def tourner_droite(y):
    return trapeze(y, 0, 15, 30, 30)

# Output fuzzy sets: linear speed (rad/s)

def vitesse_lente(y):
    return trapeze(y, 0, 0, 0.5, 1.5)

def vitesse_moyenne(y):
    return triangle(y, 1.0, 2.0, 3.0)

def vitesse_rapide(y):
    return trapeze(y, 2.5, 3.5, VMAX_FLOU, VMAX_FLOU)


def defuzzifier(regles, y_min, y_max, nb_pas=100):
    # Mamdani inference (min =AND, max = OR)
    somme_ponderee = 0.0
    somme_hauteurs = 0.0
    pas = (y_max - y_min) / nb_pas

    for i in range(nb_pas + 1):
        y = y_min + i * pas
        hauteur_max = 0.0
        for degre_activation, fonction_sortie in regles:
            h = min(degre_activation, fonction_sortie(y))
            if h > hauteur_max:
                hauteur_max = h
        somme_ponderee += y * hauteur_max
        somme_hauteurs += hauteur_max

    if somme_hauteurs == 0:
        return 0.0
    return somme_ponderee / somme_hauteurs


def controleur_flou(dist_gauche, dist_droite):
    diff = dist_gauche - dist_droite

    # 3 rules for steering control
    regles_angle = [
        (mur_plus_proche_droite(diff), tourner_gauche),
        (centre(diff), tout_droit),
        (mur_plus_proche_gauche(diff), tourner_droite),
    ]

    # 3 rules for speed control based on wall proximity
    regles_vitesse = [
        (max(proche(dist_gauche), proche(dist_droite)), vitesse_lente),
        (max(moyen(dist_gauche), moyen(dist_droite)), vitesse_moyenne),
        (min(loin(dist_gauche), loin(dist_droite)), vitesse_rapide),
    ]

    angle = defuzzifier(regles_angle, -30, 30)
    vitesse = defuzzifier(regles_vitesse, 0, VMAX_FLOU)
    return angle, vitesse

# Scale raw reading to [0, 1] and filter out background noise
def normaliser(valeur_brute):
    val = valeur_brute / VAL_MAX
    if val > 1.0:
        val = 1.0
    if val < SEUIL_BRUIT:
        return 0.0
    return val


def conversion_roues(angle, vitesse):
    v_gauche = vitesse + GAIN_ANGLE * angle
    v_droite = vitesse - GAIN_ANGLE * angle

    v_gauche = max(-VMAX_ROUE, min(VMAX_ROUE, v_gauche))
    v_droite = max(-VMAX_ROUE, min(VMAX_ROUE, v_droite))
    return v_gauche, v_droite


# Simulation
step_count = 0
while robot.step(timestep) != -1:
    ps1 = capteurs['ps1'].getValue()
    ps2 = capteurs['ps2'].getValue()
    ps5 = capteurs['ps5'].getValue()
    ps6 = capteurs['ps6'].getValue()

    # take the maximum reading on each side to represent the closest obstacle
    dist_droite = normaliser(max(ps1, ps2))
    dist_gauche = normaliser(max(ps5, ps6))

    # without ps0 and ps7, both diagonal sensors (ps1 and ps6) detect a wall straight 
    # ahead at the same time. We introduce an asymmetry so the robot slows 
    # down and initiates a right turn as soon as both sides exceed 0.06.
    if dist_gauche >= 0.06 and dist_droite >= 0.06 and abs(dist_gauche - dist_droite) < 0.05:
        dist_gauche = max(dist_gauche, 0.55)
        dist_droite = 0.0

    angle, vitesse = controleur_flou(dist_gauche, dist_droite)
    v_gauche, v_droite = conversion_roues(angle, vitesse)
    
    if step_count % 10 == 0:
        print(f"brut: ps1={ps1:.0f} ps2={ps2:.0f} ps5={ps5:.0f} ps6={ps6:.0f} | "
              f"norm: G={dist_gauche:.3f} D={dist_droite:.3f} | "
              f"angle={angle:.1f} vit={vitesse:.2f} | roues=({v_gauche:.2f}, {v_droite:.2f})")
    step_count += 1

    moteur_gauche.setVelocity(v_gauche)
    moteur_droit.setVelocity(v_droite)