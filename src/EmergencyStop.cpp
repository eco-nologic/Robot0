#include "EmergencyStop.h"

EmergencyStop& EmergencyStop::getInstance() {
    static EmergencyStop instance;
    return instance;
}

EmergencyStop::EmergencyStop() : _triggered(false) {}

void EmergencyStop::trigger() {
    _triggered = true;
}

void EmergencyStop::reset() {
    _triggered = false;
}

bool EmergencyStop::isTriggered() const {
    return _triggered;
}
