#ifndef SEQUENCE1_H
#define SEQUENCE1_H

#include "Sequence.h"

class Sequence1 : public Sequence {
public:
    Sequence1(Robot& robot) : Sequence(robot) {}
    void execute() override;
};

#endif // SEQUENCE1_H