#include "Robot.h"
#include "Telemetry.h"
#include "WifiManager.h"

/**
 * @brief Constructeur de la classe Robot.
 * Initialise les moteurs avec leurs broches respectives et configure les contrôleurs PID.
 */

// DÉFENSE : "Pourquoi utiliser une liste d'initialisation dans le constructeur ?"
// RÉPONSE : "Pour l'efficacité et la rigueur. Cela initialise les membres directement lors de leur allocation mémoire, évitant ainsi une double initialisation (construction par défaut suivie d'une affectation). C'est aussi obligatoire pour les objets comme 'Motor' qui n'ont pas de constructeur par défaut."

// DÉFENSE : "Pourquoi ces valeurs pour le PID en marche avant (2.0, 0.1, 0.5) ?"
// RÉPONSE : "C'est un compromis stabilité/réactivité. Kp=2.0 assure la correction, Ki=0.1 élimine l'erreur statique (dérive) sur la durée, et Kd=0.5 amortit les oscillations pour une trajectoire fluide."

// DÉFENSE : "Pourquoi un Kp plus élevé (5.0) et pas de I/D en marche arrière ?"
// RÉPONSE : "La marche arrière est souvent utilisée pour des corrections courtes. Un Kp élevé permet de vaincre instantanément l'inertie et le jeu mécanique. L'absence de Ki/Kd simplifie le calcul pour des mouvements où la précision brute prime sur la fluidité."

// DÉFENSE : "Pourquoi une vitesse de base (PWM) de 80 ?"
// RÉPONSE : "C'est la 'zone de confort' du robot. 80 est assez élevé pour vaincre les frottements secs sans saturer les moteurs, laissant une réserve de puissance (headroom) au PID pour accélérer une roue lors des corrections."

Robot::Robot() 
    : motorD(PIN_EN_D, PIN_IN1_D, PIN_IN2_D, PIN_ENC_D_A, false),
      motorG(PIN_EN_G, PIN_IN1_G, PIN_IN2_G, PIN_ENC_G_A, true),
      _pen({0.0f, 0.0f, 0.0f}),
      _pidForward({2.0f, 0.1f, 0.5f, 0.0f, 0.0f}),
      _pidReverse({5.0f, 0.0f, 0.0f, 0.0f, 0.0f}),
      _pwmBase(80) {}

/**
 * @brief Démarre les composants matériels du robot.
 * Doit être appelé dans le setup() d'Arduino.
 */
void Robot::begin() {
    motorD.begin();
    motorG.begin();
    Magnetometer::getInstance().begin();
    GyroAccel::getInstance().begin();
}

/**
 * @brief Coupe l'alimentation des moteurs et met à jour l'état de mouvement.
 * Notifie également la télémétrie.
 */
void Robot::stop() {
    motorD.stop();
    motorG.stop();
    Telemetry::getInstance().setMoving(false);
    Telemetry::getInstance().update(*this);
}

/**
 * @brief Calcule le cap magnétique du robot.
 * @return Angle en degrés.
 */
float Robot::getHeading() {
    return Magnetometer::getInstance().getHeading();
}

// DÉFENSE : "Pourquoi implémenter une méthode wait() personnalisée au lieu d'utiliser le delay() d'Arduino ?"
// RÉPONSE : "Pour maintenir la réactivité du système. Le delay() standard bloque le processeur, alors que cette boucle maintient le serveur Web actif et la télémétrie à jour pendant les pauses."

/**
 * @brief Remplace delay() pour permettre l'exécution des tâches de fond.
 * Gère le client web et rafraîchit la télémétrie pendant l'attente.
 * @param ms Durée de l'attente.
 */
void Robot::wait(uint32_t ms) {
    uint32_t start = millis();
    while (millis() - start < ms) {
        if (EmergencyStop::getInstance().isTriggered()) {
            stop();
            return;
        }
        WifiManager::getInstance().update();
        Telemetry::getInstance().update(*this);
        delay(10);
    }
}

/**
 * @brief Normalise un angle dans l'intervalle [-PI, PI].
 * @param angle Angle en radians.
 * @return Angle normalisé.
 */
float Robot::normalizeAngle(float angle) {
    while (angle > PI) angle -= 2.0 * PI;
    while (angle < -PI) angle += 2.0 * PI;
    return angle;
}

// DÉFENSE : "Pourquoi calculer l'odométrie par rapport au stylo et non au centre de l'essieu ?"
// RÉPONSE : "Puisque le but du robot est de dessiner, nous devons suivre la trace laissée par le stylo. Comme celui-ci est déporté (offset), sa trajectoire dans les virages diffère de celle du centre des roues."

/**
 * @brief Algorithme de mise à jour de la pose (position et angle).
 * Prend en compte le déport (offset) du stylo par rapport à l'axe des roues.
 * @param distance_center Distance parcourue par le centre de l'essieu.
 * @param delta_theta Variation de l'angle d'orientation.
 */
void Robot::updateOdometry(float distance_center, float delta_theta) {
    _pen.theta = normalizeAngle(_pen.theta + delta_theta);
    float rear_x = _pen.x - ROBOT_STYLO_OFFSET * cosf(_pen.theta);
    float rear_y = _pen.y - ROBOT_STYLO_OFFSET * sinf(_pen.theta);
    rear_x += distance_center * cosf(_pen.theta);
    rear_y += distance_center * sinf(_pen.theta);
    _pen.x = rear_x + ROBOT_STYLO_OFFSET * cosf(_pen.theta);
    _pen.y = rear_y + ROBOT_STYLO_OFFSET * sinf(_pen.theta);
}

/**
 * @brief Effectue une rotation précise basée sur les encodeurs.
 * @param angleDeg Angle cible.
 * @param speed Puissance moteur.
 */
void Robot::rotate(float angleDeg, int speed) {
    float L = ROBOT_ENTRAXE;
    float rayonRotation = L / 2.0;
    float arc = 2 * PI * rayonRotation * (abs(angleDeg) / 360.0);
    long ticks = (arc / ROBOT_ROUE_PERIMETRE) * ROBOT_TICKS_PAR_ROTATION;

    // Set theoretical targets for telemetry validation
    Telemetry::getInstance().setTargetCommand(0.0f, angleDeg);
    Telemetry::getInstance().setMoving(true);

    motorD.resetTicks();
    motorG.resetTicks();

    while ((abs(motorD.getTicks()) + abs(motorG.getTicks())) / 2 < ticks) {
        if (EmergencyStop::getInstance().isTriggered()) {
            stop();
            return;
        }
        if (angleDeg > 0) {
            motorG.drive(-speed);
            motorD.drive(speed);
        } else {
            motorG.drive(speed);
            motorD.drive(-speed);
        }
        Telemetry::getInstance().update(*this);
        delay(10);
    }
    stop();
    updateOdometry(0.0, angleDeg * PI / 180.0);
}

/**
 * @brief Avance en ligne droite avec correction PID.
 * Utilise la différence entre les encodeurs gauche et droit pour corriger la trajectoire.
 * @param cm Distance à parcourir.
 */
void Robot::moveForward(float cm) {
    int ticks_target = (int)((cm / ROBOT_ROUE_PERIMETRE) * ROBOT_TICKS_PAR_ROTATION);

    Telemetry::getInstance().setTargetCommand(cm, 0.0f);
    Telemetry::getInstance().setMoving(true);

    motorD.resetTicks();
    motorG.resetTicks();
    _pidForward.error_prev = 0;
    _pidForward.integral = 0;

    while (motorD.getTicks() < ticks_target || motorG.getTicks() < ticks_target) {
        if (EmergencyStop::getInstance().isTriggered()) {
            stop();
            return;
        }
        long error = motorG.getTicks() - motorD.getTicks();
        _pidForward.integral = constrain(_pidForward.integral + error, -500, 500);
        float derivative = error - _pidForward.error_prev;
        float correction = _pidForward.Kp * error + _pidForward.Ki * _pidForward.integral + _pidForward.Kd * derivative;
        _pidForward.error_prev = error;

        motorD.drive(_pwmBase + correction);
        motorG.drive(_pwmBase);
        Telemetry::getInstance().update(*this);
        delay(10);
    }
    stop();
    updateOdometry(cm, 0.0);
}

/**
 * @brief Recule en ligne droite avec asservissement.
 * @param cm Distance à parcourir.
 */
void Robot::moveBackward(float cm) {
    int ticks_target = (int)((cm / ROBOT_ROUE_PERIMETRE) * ROBOT_TICKS_PAR_ROTATION);

    Telemetry::getInstance().setTargetCommand(-cm, 0.0f);
    Telemetry::getInstance().setMoving(true);

    motorD.resetTicks();
    motorG.resetTicks();
    _pidReverse.error_prev = 0;
    _pidReverse.integral = 0;

    while (motorD.getTicks() < ticks_target && motorG.getTicks() < ticks_target) {
        if (EmergencyStop::getInstance().isTriggered()) {
            stop();
            return;
        }
        long error = motorG.getTicks() - motorD.getTicks();
        _pidReverse.integral = constrain(_pidReverse.integral + error, -500, 500);
        motorD.drive(-(_pwmBase + (_pidReverse.Kp * error)));
        motorG.drive(-_pwmBase);
        Telemetry::getInstance().update(*this);
        delay(10);
    }
    stop();
    updateOdometry(-cm, 0.0);
}