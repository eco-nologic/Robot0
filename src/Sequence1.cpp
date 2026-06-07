#include "Sequence1.h"

void Sequence1::execute() {
    _robot.wait(2000);
    _robot.moveForward(20);
    _robot.wait(500);
    _robot.rotate(90);
    _robot.wait(500);
    _robot.moveForward(10);
    _robot.wait(500);
    _robot.rotate(-90);
    _robot.wait(500);
    _robot.moveForward(40);
    _robot.stop();
}