#ifndef MAGNETOMETER_H
#define MAGNETOMETER_H

#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>
#include "GyroAccel.h"

// DÉFENSE : "Pourquoi utiliser un Singleton pour le magnétomètre ?"
// RÉPONSE : "Pour garantir un accès unique et global à la ressource matérielle I2C, évitant ainsi les conflits d'initialisation et facilitant le partage des données de calibration entre les différentes séquences."

// DÉFENSE : "À quoi sert le paramètre scaleY ?"
// RÉPONSE : "C'est une correction de type 'Soft-Iron'. Elle compense les distorsions du champ magnétique causées par les composants métalliques du robot, transformant une lecture elliptique en un cercle parfait pour un calcul de cap précis."

/**
 * @brief Singleton class to manage the LIS3MDL Magnetometer hardware and calibration.
 */
class Magnetometer {
public:
    // Singleton access
    static Magnetometer& getInstance();

    // Prevent copying
    Magnetometer(const Magnetometer&) = delete;
    Magnetometer& operator=(const Magnetometer&) = delete;

    /** @brief Initialise le capteur LIS3MDL sur le bus I2C. */
    bool begin();
    
    /** @brief Réinitialise les offsets de calibration. */
    void resetCalibration();
    /** @brief Met à jour les bornes min/max pour l'auto-calibration. */
    void updateLimits(float x, float y, float z);
    /** @brief Calcule les offsets finaux après une phase de rotation. */
    void finalizeCalibration(float minX, float maxX, float minY, float maxY);
    
    /** @brief Calcule le cap actuel compensé. */
    float getHeading();

    /** @brief Indique si le capteur a été calibré. */
    bool isCalibrated() const { return calibrated; }
    /** @brief Offset axe X. */
    float getOffsetX() const { return offsetX; }
    /** @brief Offset axe Y. */
    float getOffsetY() const { return offsetY; }
    /** @brief Facteur d'échelle Soft-Iron. */
    float getScaleY() const { return scaleY; }

    Adafruit_LIS3MDL& getSensor() { return lis3mdl; }

private:
    Magnetometer(GyroAccel& ga); // Private constructor for singleton

    Adafruit_LIS3MDL lis3mdl;       // Magnétomètre
    GyroAccel& gyroAccel;           // Reference to the IMU for heading offset
    float offsetX = 0;
    float offsetY = 0;
    float offsetZ = 0;
    float scaleY = 1.0;  // Correction soft-iron (ratio des amplitudes X/Y)
    bool calibrated = false;
};

#endif // MAGNETOMETER_H