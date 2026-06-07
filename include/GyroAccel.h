#ifndef GYRO_ACCEL_H
#define GYRO_ACCEL_H

#include <SparkFunLSM6DS3.h>

// DÉFENSE : "Pourquoi stocker un headingOffset dans cette classe ?"
// RÉPONSE : "Cela permet de décorréler l'orientation physique du capteur sur le PCB du vecteur 'avant' réel du robot. On peut ainsi ajuster l'alignement logiciellement sans modifier la mécanique."

/**
 * @brief Singleton class to manage the LSM6DS3 Gyroscope/Accelerometer hardware.
 */
class GyroAccel {
public:
    // Singleton access
    static GyroAccel& getInstance();

    // Prevent copying
    GyroAccel(const GyroAccel&) = delete;
    GyroAccel& operator=(const GyroAccel&) = delete;

    // Hardware management
    bool begin();

    // Access to the raw sensor
    LSM6DS3& getImu() { return imu; }

    // Heading offset management
    float getHeadingOffset() const { return headingOffset; }
    void setHeadingOffset(float offset) { headingOffset = offset; }
    void saveHeadingOffsetToEEPROM();
    void loadHeadingOffsetFromEEPROM();

private:
    // Private constructor for singleton
    GyroAccel();

    LSM6DS3 imu; // Magnétomètre renamed to imu

    /**
     * Offset de montage du capteur (en degrés)
     * Le capteur n'est pas aligné avec l'avant du robot
     * Calibrated value: -90° (compass reads 90° high, needs -90° correction)
     * Can be overridden via GUI "Set & Save to Robot" → saves to EEPROM
     */
    float headingOffset = -90.0f;
};

#endif // GYRO_ACCEL_H