#include "Magnetometer.h"
#include <Arduino.h>

Magnetometer& Magnetometer::getInstance() {
    static Magnetometer instance(GyroAccel::getInstance());
    return instance;
}

Magnetometer::Magnetometer(GyroAccel& ga) : gyroAccel(ga) {}

bool Magnetometer::begin() {
    if (!lis3mdl.begin_I2C(0x1E)) {
        return false;
    }
    lis3mdl.setPerformanceMode(LIS3MDL_ULTRAHIGHMODE);
    lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
    lis3mdl.setDataRate(LIS3MDL_DATARATE_10_HZ);
    lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
    return true;
}

void Magnetometer::resetCalibration() {
    offsetX = 0;
    offsetY = 0;
    offsetZ = 0;
    scaleY = 1.0;
    calibrated = false;
}

void Magnetometer::finalizeCalibration(float minX, float maxX, float minY, float maxY) {
    offsetX = (maxX + minX) / 2.0f;
    offsetY = (maxY + minY) / 2.0f;
    float rangeX = maxX - minX;
    float rangeY = maxY - minY;
    scaleY = (rangeY > 0.1f) ? (rangeX / rangeY) : 1.0f;
    calibrated = true;
}

float Magnetometer::getHeading() {
    sensors_event_t mag;
    lis3mdl.getEvent(&mag);

    float mx_cal = mag.magnetic.x - offsetX;
    float my_cal = (mag.magnetic.y - offsetY) * scaleY;  // Correction soft-iron

    float heading = atan2(my_cal, mx_cal) * 180.0f / PI;
    if (heading < 0) heading += 360.0f;
    
    // Appliquer l'offset de montage du capteur
    heading += gyroAccel.getHeadingOffset();
    
    // Normalisation 0-360
    while (heading < 0) heading += 360.0f;
    while (heading >= 360.0f) heading -= 360.0f;
    
    return heading;
}