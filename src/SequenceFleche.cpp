#include "SequenceFleche.h"

void SequenceFleche::execute() {
    _robot.moveForward(10);
    _robot.wait(500);
    // Low level movements via test utility would go here or be refactored
}