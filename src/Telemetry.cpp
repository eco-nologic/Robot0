#include "Telemetry.h"
#include "Magnetometer.h"
#include "GyroAccel.h"

Telemetry& Telemetry::getInstance() {
    static Telemetry instance;
    return instance;
}

Telemetry::Telemetry() 
    : _lastUpdateTime(0), 
      _lastLeftTicks(0), 
      _lastRightTicks(0) 
{
    _mutex = xSemaphoreCreateMutex();
    memset(&_data, 0, sizeof(TelemetryData));
}

void Telemetry::update(Robot& robot) {
    // DÉFENSE : "Pourquoi limiter la fréquence de mise à jour à 50Hz ?"
    // RÉPONSE : "Pour économiser des cycles CPU et de la bande passante I2C. L'interrogation à haute fréquence de capteurs comme le magnétomètre offre des rendements décroissants et peut affamer la boucle de contrôle des moteurs."
    
    uint32_t currentTime = millis();

    // Frequency check: return if the last update was too recent
    if (_lastUpdateTime != 0 && (currentTime - _lastUpdateTime < MIN_UPDATE_INTERVAL_MS)) return;

    // Try to take the mutex. If taken, return immediately to avoid blocking.
    if (xSemaphoreTake(_mutex, 0) != pdTRUE) return;

    uint32_t dt = currentTime - _lastUpdateTime;
    
    _data.timestamp = currentTime;

    RobotPen& pen = robot.getPen();
    Magnetometer& magSens = Magnetometer::getInstance();
    GyroAccel& imuSens = GyroAccel::getInstance();
    LSM6DS3& imuRaw = imuSens.getImu();

    // Pose
    _data.penX = pen.x;
    _data.penY = pen.y;
    _data.robotHeading = pen.theta;
    _data.compassOffset = imuSens.getHeadingOffset();
    _data.heading = magSens.getHeading();

    long currentLeftTicks = robot.motorG.getTicks();
    long currentRightTicks = robot.motorD.getTicks();

    // Moteurs
    _data.leftWheelSteps = currentLeftTicks;
    _data.rightWheelSteps = currentRightTicks;

    if (dt > 0 && _lastUpdateTime > 0) {
        // Distance per tick in mm: (Circumference in cm * 10) / TicksPerRotation
        float distancePerTickMM = (ROBOT_ROUE_PERIMETRE * 10.0f) / (float)ROBOT_TICKS_PAR_ROTATION;
        
        // Speed = (delta_ticks * distance_per_tick) / delta_time_in_seconds
        _data.leftWheelSpeed = ((float)(currentLeftTicks - _lastLeftTicks) * distancePerTickMM) / ((float)dt / 1000.0f);
        _data.rightWheelSpeed = ((float)(currentRightTicks - _lastRightTicks) * distancePerTickMM) / ((float)dt / 1000.0f);
        
        // Calculate actual linear (V) and angular (W) velocities for validation (travail.pdf)
        _data.actualV = (_data.leftWheelSpeed + _data.rightWheelSpeed) / 2.0f;
        _data.actualW = (_data.rightWheelSpeed - _data.leftWheelSpeed) / (ROBOT_ENTRAXE * 10.0f);
        
        // Update movement status: considered moving if linear speed > 0.1mm/s 
        // or angular speed > 0.01 rad/s
        _data.isMoving = (abs(_data.actualV) > 0.1f || abs(_data.actualW) > 0.01f);
    }

    _lastUpdateTime = currentTime;
    _lastLeftTicks = currentLeftTicks;
    _lastRightTicks = currentRightTicks;

    // IMU Brutes
    _data.accelX = imuRaw.readFloatAccelX();
    _data.accelY = imuRaw.readFloatAccelY();
    _data.accelZ = imuRaw.readFloatAccelZ();
    _data.gyroX = imuRaw.readFloatGyroX();
    _data.gyroY = imuRaw.readFloatGyroY();
    _data.gyroZ = imuRaw.readFloatGyroZ();
    
    sensors_event_t magEvent;
    magSens.getSensor().getEvent(&magEvent);
    _data.magX = magEvent.magnetic.x;
    _data.magY = magEvent.magnetic.y;
    _data.magZ = magEvent.magnetic.z;

    // État
    _data.isCalibrated = magSens.isCalibrated();
    _data.batteryVoltage = 0.0f; // To be implemented with ADC if PIN_BATTERY_ADC is used

    xSemaphoreGive(_mutex);
}

String Telemetry::getJSON() {
    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(100)) != pdTRUE) return "{}";

    String json = "{";
    json += "\"penX\":" + String(_data.penX) + ",";
    json += "\"penY\":" + String(_data.penY) + ",";
    json += "\"heading\":" + String(_data.heading) + ",";
    json += "\"isMoving\":" + String(_data.isMoving ? "true" : "false") + ",";
    json += "\"battery\":" + String(_data.batteryVoltage) + ",";
    
    // IMU
    json += "\"imu\":{";
    json += "\"accel\":[" + String(_data.accelX) + "," + String(_data.accelY) + "," + String(_data.accelZ) + "],";
    json += "\"gyro\":[" + String(_data.gyroX) + "," + String(_data.gyroY) + "," + String(_data.gyroZ) + "]";
    json += "},";

    // Ticks
    json += "\"ticks\":{";
    json += "\"L\":" + String(_data.leftWheelSteps) + ",";
    json += "\"R\":" + String(_data.rightWheelSteps);
    json += "},";

    // Validation
    json += "\"validation\":{";
    json += "\"targetL\":" + String(_data.targetL) + ",";
    json += "\"actualV\":" + String(_data.actualV) + ",";
    json += "\"targetW\":" + String(_data.targetW) + ",";
    json += "\"actualW\":" + String(_data.actualW);
    json += "}";

    json += "}";
    xSemaphoreGive(_mutex);
    return json;
}

String Telemetry::getCSVHeader() {
    return "timestamp,penX,penY,robotHeading,heading,L_steps,R_steps,accX,accY,accZ,gyroX,gyroY,gyroZ,magX,magY,magZ,batt,isMoving,targetL,actualV,targetW,actualW";
}

String Telemetry::getCSV() {
    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(100)) != pdTRUE) return "";

    String csv = "";
    csv += String(_data.timestamp) + ",";
    csv += String(_data.penX) + ",";
    csv += String(_data.penY) + ",";
    csv += String(_data.robotHeading) + ",";
    csv += String(_data.heading) + ",";
    csv += String(_data.leftWheelSteps) + ",";
    csv += String(_data.rightWheelSteps) + ",";
    
    // IMU
    csv += String(_data.accelX) + ",";
    csv += String(_data.accelY) + ",";
    csv += String(_data.accelZ) + ",";
    csv += String(_data.gyroX) + ",";
    csv += String(_data.gyroY) + ",";
    csv += String(_data.gyroZ) + ",";
    
    // Mag
    csv += String(_data.magX) + ",";
    csv += String(_data.magY) + ",";
    csv += String(_data.magZ) + ",";
    
    // State
    csv += String(_data.batteryVoltage) + ",";
    csv += String(_data.isMoving ? "1" : "0") + ",";
    
    // Validation
    csv += String(_data.targetL) + ",";
    csv += String(_data.actualV) + ",";
    csv += String(_data.targetW) + ",";
    csv += String(_data.actualW);
    
    xSemaphoreGive(_mutex);
    return csv;
}

TelemetryData Telemetry::getData() {
    TelemetryData copy;
    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        copy = _data;
        xSemaphoreGive(_mutex);
    }
    return copy;
}

void Telemetry::setMoving(bool moving) {
    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        _data.isMoving = moving;
        xSemaphoreGive(_mutex);
    }
}

void Telemetry::setWaypoint(int index, float tx, float ty) {
    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        _data.waypointIndex = index;
        _data.targetX = tx;
        _data.targetY = ty;
        xSemaphoreGive(_mutex);
    }
}

void Telemetry::setBearingToTarget(float bearing) {
    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        _data.bearingToTarget = bearing;
        xSemaphoreGive(_mutex);
    }
}

void Telemetry::setTargetCommand(float l, float theta, float r) {
    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        _data.targetL = l;
        _data.targetTheta = theta;
        _data.targetR = r;
        xSemaphoreGive(_mutex);
    }
}

void Telemetry::setTargetSpeeds(float v, float w) {
    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        _data.targetV = v;
        _data.targetW = w;
        xSemaphoreGive(_mutex);
    }
}

void Telemetry::setActualSpeeds(float v, float w) {
    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        _data.actualV = v;
        _data.actualW = w;
        xSemaphoreGive(_mutex);
    }
}