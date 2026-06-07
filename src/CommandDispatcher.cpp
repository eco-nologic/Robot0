#include "CommandDispatcher.h"
#include "Robot.h"
#include "Sequences.h"
#include "EmergencyStop.h"
#include "MotorUtility.h"

CommandDispatcher& CommandDispatcher::getInstance() {
    static CommandDispatcher instance;
    return instance;
}

CommandDispatcher::CommandDispatcher() : _robot(nullptr) {}

void CommandDispatcher::begin(Robot& robot) {
    _robot = &robot;
}

void CommandDispatcher::dispatch(const String& cmd) {
    if (!_robot) {
        Serial.println("CommandDispatcher: robot not initialized");
        return;
    }

    String upperCmd = cmd;
    upperCmd.toUpperCase();

    // Sequence commands
    if (upperCmd == "S1" || upperCmd == "SEQUENCE1") {
        EmergencyStop::getInstance().reset();
        Sequence1 s(*_robot);
        s.execute();
        return;
    }

    if (upperCmd == "S2" || upperCmd == "SEQUENCE2") {
        EmergencyStop::getInstance().reset();
        Sequence2 s(*_robot);
        s.execute();
        return;
    }

    if (upperCmd == "S3" || upperCmd == "SEQUENCE3") {
        EmergencyStop::getInstance().reset();
        Sequence3 s(*_robot);
        s.execute();
        return;
    }

    if (upperCmd == "FLECHE") {
        EmergencyStop::getInstance().reset();
        SequenceFleche s(*_robot);
        s.execute();
        return;
    }

    // Emergency stop
    if (upperCmd == "STOP") {
        EmergencyStop::getInstance().trigger();
        _robot->stop();
        return;
    }

    // Angle droit sequence (hardcoded from handleAngleDroit)
    if (upperCmd == "ANGLE_DROIT") {
        EmergencyStop::getInstance().reset();
        bouger_ticks_en_1s(353, 353, 80, 80, 0.0, 0.0);
        bouger_ticks_en_1s(30, -10, 100, 100, 0.0, 0.0);
        bouger_ticks_en_1s(50, -10, 100, 100, 0.0, 0.0);
        bouger_ticks_en_1s(50, 1, 100, 70, 0.0, 0.0);
        bouger_ticks_en_1s(50, 1, 100, 70, 0.0, 0.4);
        bouger_ticks_en_1s(30, 50, 100, 100, 0.0, 0.0, 4.00);
        bouger_ticks_en_1s(33, 50, 100, 100, 0.0, 0.0, 3.94);
        bouger_ticks_en_1s(35, 50, 100, 100, 0.0, 0.0, 3.89);
        bouger_ticks_en_1s(37, 50, 100, 100, 0.0, 0.0, 3.83);
        bouger_ticks_en_1s(37, 50, 100, 100, 0.0, 0.0, 3.78);
        bouger_ticks_en_1s(39, 50, 100, 100, 0.0, 0.0, 3.72);
        bouger_ticks_en_1s(39, 50, 100, 100, 0.0, 0.0, 3.67);
        bouger_ticks_en_1s(41, 50, 100, 100, 0.0, 0.0, 3.61);
        bouger_ticks_en_1s(41, 50, 100, 100, 0.0, 0.0, 3.56);
        bouger_ticks_en_1s(43, 50, 100, 100, 0.0, 0.0, 3.50);
        bouger_ticks_en_1s(43, 50, 100, 100, 0.0, 0.0, 3.44);
        bouger_ticks_en_1s(44, 50, 100, 100, 0.0, 0.0, 3.39);
        bouger_ticks_en_1s(45, 50, 100, 100, 0.0, 0.0, 3.33);
        bouger_ticks_en_1s(45, 50, 100, 100, 0.0, 0.0, 3.28);
        bouger_ticks_en_1s(45, 50, 100, 100, 0.0, 0.0, 3.22);
        bouger_ticks_en_1s(46, 50, 100, 100, 0.0, 0.0, 3.17);
        bouger_ticks_en_1s(46, 50, 100, 100, 0.0, 0.0, 3.11);
        bouger_ticks_en_1s(46, 50, 100, 100, 0.0, 0.0, 3.06);
        bouger_ticks_en_1s(47, 50, 100, 100, 0.0, 0.0, 3.00);
        return;
    }

    Serial.print("CommandDispatcher: Unknown command: ");
    Serial.println(cmd);
}
