#ifndef ROBOT_H
#define ROBOT_H

#include "ConfigHardware.h"
#include "Motor.h"
#include "RobotPen.h"
#include "PIDConfig.h"
#include "Magnetometer.h"
#include "GyroAccel.h"
#include "EmergencyStop.h"

// DÉFENSE : "Pourquoi tout encapsuler dans une classe Robot ?"
// RÉPONSE : "Pour créer une couche d'abstraction matérielle (HAL). Cela découple la logique de dessin de haut niveau de la manipulation des registres/PWM de bas niveau."

/**
 * @brief Classe principale représentant le robot.
 * Gère l'agrégation des moteurs, des capteurs et de l'odométrie.
 */
class Robot {
public:
    /** @brief Constructeur initialisant les moteurs et les PID. */
    Robot();
    
    /** @brief Initialise les périphériques matériels (I2C, Moteurs, Capteurs). */
    void begin();
    
    /** @brief Avance d'une distance donnée en utilisant un asservissement PID. @param cm Distance en centimètres. */
    void moveForward(float cm);
    /** @brief Recule d'une distance donnée. @param cm Distance en centimètres. */
    void moveBackward(float cm);
    /** @brief Effectue une rotation sur place. @param angleDeg Angle en degrés (positif = gauche). @param speed Vitesse PWM. */
    void rotate(float angleDeg, int speed = 80);
    /** @brief Arrête immédiatement les deux moteurs. */
    void stop();
    /** @brief Attente active permettant de garder le serveur web et la télémétrie actifs. @param ms Temps en millisecondes. */
    void wait(uint32_t ms);
    
    /** @brief Récupère le cap actuel via le magnétomètre. @return Cap en degrés [0, 360[. */
    float getHeading();
    /** @brief Accès à l'objet de positionnement (Stylo). */
    RobotPen& getPen() { return _pen; }
    /** @brief Définit la vitesse PWM de base pour les déplacements. */
    void setPwmBase(int pwm) { _pwmBase = pwm; }
    /** @brief Récupère la vitesse PWM de base. */
    int getPwmBase() const { return _pwmBase; }
    /** @brief Met à jour les coordonnées X, Y et l'orientation du stylo. */
    void updateOdometry(float distance_center, float delta_theta);

    // Composants publics pour accès direct (ex: Interruptions)
    Motor motorD;
    Motor motorG;

private:
    float normalizeAngle(float angle);

    RobotPen _pen;
    PIDController _pidForward;
    PIDController _pidReverse;
    int _pwmBase;
};

#endif // ROBOT_H