#include "CommandDispatcher.h"
#include "Robot.h"
#include "Sequences.h"
#include "EmergencyStop.h"
#include "MotorUtility.h"
#include "IKMove.h"
#include "SequenceCorner90.h"

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

    if (upperCmd == "CORNER90") {
        SequenceCorner90 s(*_robot);
        s.execute();
        return;
    }

    // Angle droit sequence (hardcoded from handleAngleDroit)
    if (upperCmd == "ANGLE_DROIT") {
        executeCorner90Logic();
        return;
    }

    // PID Tuning commands
    if (upperCmd.startsWith("PID_FWD:")) {
        String params = upperCmd.substring(8);  // Extract "2.0,0.1,0.5"
        int comma1 = params.indexOf(',');
        int comma2 = params.indexOf(',', comma1 + 1);

        if (comma1 > 0 && comma2 > comma1) {
            float kp = params.substring(0, comma1).toFloat();
            float ki = params.substring(comma1 + 1, comma2).toFloat();
            float kd = params.substring(comma2 + 1).toFloat();
            _robot->setPidForward(kp, ki, kd);
        }
        return;
    }

    if (upperCmd.startsWith("PID_REV:")) {
        String params = upperCmd.substring(8);
        int comma1 = params.indexOf(',');
        int comma2 = params.indexOf(',', comma1 + 1);

        if (comma1 > 0 && comma2 > comma1) {
            float kp = params.substring(0, comma1).toFloat();
            float ki = params.substring(comma1 + 1, comma2).toFloat();
            float kd = params.substring(comma2 + 1).toFloat();
            _robot->setPidReverse(kp, ki, kd);
        }
        return;
    }

    if (upperCmd.startsWith("PWM_BASE:")) {
        String pwmStr = upperCmd.substring(9);
        int pwm = pwmStr.toInt();
        if (pwm > 0 && pwm <= 255) {
            _robot->setPwmBase(pwm);
        }
        return;
    }

    if (upperCmd == "PID_RESET") {
        _robot->setPidForward(2.0f, 0.1f, 0.5f);
        _robot->setPidReverse(5.0f, 0.0f, 0.0f);
        _robot->setPwmBase(80);
        Serial.println("PID values reset to defaults");
        return;
    }

    // IK Move command — execute one 1-second segment
    if (upperCmd.startsWith("IK_MOVE:")) {
        IKMove m = IKMove::fromParams(upperCmd.substring(8));
        bouger_ticks_en_1s(m.tg, m.td, m.vg, m.vd, m.dg, m.dd);
        return;
    }

    // Heading offset command — set and save to EEPROM
    if (upperCmd.startsWith("SET_HEADING_OFFSET:")) {
        float offset = cmd.substring(19).toFloat();
        GyroAccel::getInstance().setHeadingOffset(offset);
        GyroAccel::getInstance().saveHeadingOffsetToEEPROM();
        Serial.print("Heading offset set to ");
        Serial.print(offset);
        Serial.println(" degrees (saved to EEPROM)");
        return;
    }

    Serial.print("CommandDispatcher: Unknown command: ");
    Serial.println(cmd);
}

void CommandDispatcher::executeCorner90Logic() {
    EmergencyStop::getInstance().reset();
    bouger_ticks_en_1s(353, 353, 80, 80, 0.0, 0.0);
    bouger_ticks_en_1s(30, -10, 100, 100, 0.0, 0.0);
    bouger_ticks_en_1s(50, -10, 100, 100, 0.0, 0.0);
    bouger_ticks_en_1s(50, 1, 100, 70, 0.0, 0.5);
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
    if (_robot) _robot->stop();
}
