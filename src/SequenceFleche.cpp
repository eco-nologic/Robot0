#include "SequenceFleche.h"
#include "MotorUtility.h"

void SequenceFleche::execute() {
    // 1. Rotate to face North
    _robot.rotateToNorth(80);
    _robot.wait(500);

    // 2. Move forward
    _robot.moveForward(10);
    _robot.wait(500);

    // 3. Draw arrow pattern using tick-based movements
    bouger_ticks_en_1s(-16, 20, 100, 100, 0.0, 0.0, 4.0);
    bouger_ticks_en_1s(50, 20, 120, 120, 0.0, 0.0, 4.0);
    bouger_ticks_en_1s(50, 20, 120, 120, 0.0, 0.0, 4.0);
    bouger_ticks_en_1s(-20, -40, 120, 120, 0.0, 0.0, 4.0);
    bouger_ticks_en_1s(-20, -40, 120, 120, 0.0, 0.0, 4.0);
    bouger_ticks_en_1s(-20, 20, 100, 100, 0.0, 0.0, 4.0);
    bouger_ticks_en_1s(-10, 20, 100, 100, 0.0, 0.0, 4.0);
    bouger_ticks_en_1s(40, -20, 100, 100, 0.0, 0.0, 4.0);
    bouger_ticks_en_1s(-20, 30, 100, 100, 0.0, 0.0, 4.0);
    bouger_ticks_en_1s(28, -15, 100, 100, 0.0, 0.0, 4.0);
    bouger_ticks_en_1s(-16, 22, 100, 100, 0.0, 0.0, 4.0);
    bouger_ticks_en_1s(22, -10, 100, 100, 0.0, 0.0, 4.0);
    bouger_ticks_en_1s(-12, 15, 100, 100, 0.0, 0.0, 4.0);
    bouger_ticks_en_1s(15, -8, 100, 100, 0.0, 0.0, 4.0);
    bouger_ticks_en_1s(-8, 12, 100, 100, 0.0, 0.0, 4.0);
    bouger_ticks_en_1s(12, -6, 100, 100, 0.0, 0.0, 4.0);
    bouger_ticks_en_1s(-5, 10, 100, 100, 0.0, 0.0, 4.0);
    bouger_ticks_en_1s(8, -4, 100, 100, 0.0, 0.0, 4.0);
    bouger_ticks_en_1s(-3, 6, 100, 100, 0.0, 0.0, 4.0);
}