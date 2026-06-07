#ifndef SEQUENCE3_H
#define SEQUENCE3_H

#include "Sequence.h"

class Sequence3 : public Sequence {
public:
    Sequence3(Robot& robot) : Sequence(robot) {}
    void execute() override;
};

#endif // SEQUENCE3_H