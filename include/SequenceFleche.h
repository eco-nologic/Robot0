#ifndef SEQUENCE_FLECHE_H
#define SEQUENCE_FLECHE_H

#include "Sequence.h"

class SequenceFleche : public Sequence {
public:
    SequenceFleche(Robot& robot) : Sequence(robot) {}
    void execute() override;
};

#endif // SEQUENCE_FLECHE_H