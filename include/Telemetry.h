#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "Robot.h"

// DÉFENSE : "Pourquoi utiliser une structure séparée pour TelemetryData ?"
// RÉPONSE : "Pour permettre des captures atomiques. Nous pouvons copier l'état complet sous un verrou mutex pour garantir la cohérence des données lors de la génération de sorties JSON ou CSV."

// DÉFENSE : "Pourquoi utiliser un Mutex (Sémaphore) dans la Télémétrie ?"
// RÉPONSE : "Pour éviter les conditions de concurrence (race conditions). La fonction update() s'exécute dans la boucle principale tandis que getJSON() est appelé par la tâche du serveur Web ; le mutex garantit que nous ne lisons pas de valeurs partiellement écrites."

struct TelemetryData {
    uint32_t timestamp;
    // Pose du robot
    float penX;
    float penY;
    float robotHeading;
    float compassOffset;
    float heading;

    // Moteurs
    float leftWheelSpeed;
    float rightWheelSpeed;
    long leftWheelSteps;
    long rightWheelSteps;
    
    // IMU (données brutes)
    float accelX, accelY, accelZ;
    float gyroX, gyroY, gyroZ;
    float magX, magY, magZ;

    // Autres
    float batteryVoltage;
    bool isMoving;
    bool isCalibrated;
    int waypointIndex;
    float targetX, targetY;
    float bearingToTarget;

    // Valeurs Théoriques
    float targetL;      // Distance théorique demandée (lth)
    float targetTheta;  // Angle théorique demandé (theth)
    float targetR;      // Rayon théorique (rth)
    float targetV;      // Consigne linéaire (mm/s)
    float targetW;      // Consigne angulaire (rad/s)
    float actualV;      // Vitesse linéaire mesurée (mm/s)
    float actualW;      // Vitesse angulaire mesurée (rad/s)
};

/**
 * @brief Singleton gérant la collecte et l'exposition des données système.
 * Protégé par Mutex pour la sécurité multi-tâches (Main / WebServer).
 */
class Telemetry {
public:
    /** @brief Accès à l'instance unique de télémétrie. */
    static Telemetry& getInstance();
    Telemetry(const Telemetry&) = delete;
    Telemetry& operator=(const Telemetry&) = delete;

    /** @brief Échantillonne les données du robot et des capteurs. @param robot Instance du robot à sonder. */
    void update(Robot& robot);
    /** @brief Génère une chaîne JSON contenant l'état actuel. */
    String getJSON();
    /** @brief Génère une ligne CSV pour le datalogging. */
    String getCSV();
    /** @brief Génère l'en-tête CSV. */
    String getCSVHeader();

    /** @brief Retourne une copie sécurisée des données de télémétrie. */
    TelemetryData getData();
    
    /** @brief Définit si le robot est considéré en mouvement. */
    void setMoving(bool moving);
    /** @brief Définit l'index du point de passage et la cible. */
    void setWaypoint(int index, float tx, float ty);
    /** @brief Définit l'angle vers la cible. */
    void setBearingToTarget(float bearing);
    /** @brief Définit les commandes théoriques (Validation travail.pdf). */
    void setTargetCommand(float l, float theta, float r = 0);
    /** @brief Définit les vitesses de consigne. */
    void setTargetSpeeds(float v, float w);
    /** @brief Définit les vitesses réelles mesurées. */
    void setActualSpeeds(float v, float w);

private:
    Telemetry(); // Private constructor for singleton

    static const uint32_t MIN_UPDATE_INTERVAL_MS = 20; // 50Hz frequency limit

    TelemetryData _data;
    SemaphoreHandle_t _mutex;
    uint32_t _lastUpdateTime;
    long _lastLeftTicks;
    long _lastRightTicks;
};

#endif // TELEMETRY_H