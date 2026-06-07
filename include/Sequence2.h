#ifndef SEQUENCE2_H
#define SEQUENCE2_H

#include "Sequence.h"

class Sequence2 : public Sequence {
public:
    Sequence2(Robot& robot) : Sequence(robot) {}
    void execute() override;
};

#endif // SEQUENCE2_H