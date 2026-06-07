#ifndef EMERGENCY_STOP_H
#define EMERGENCY_STOP_H

#include <Arduino.h>

class EmergencyStop {
public:
    static EmergencyStop& getInstance();

    void trigger();
    void reset();
    bool isTriggered() const;

    EmergencyStop(const EmergencyStop&) = delete;
    EmergencyStop& operator=(const EmergencyStop&) = delete;

private:
    EmergencyStop();
    bool _triggered;
};

#endif // EMERGENCY_STOP_H
