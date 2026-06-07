#ifndef SEQUENCE1_H
#define SEQUENCE1_H

#include "Sequence.h"

/**
 * @brief Séquence 1 : Parcours en escalier.
 * Exécute une série de lignes droites et de virages à 90° pour former un escalier.
 */
class Sequence1 : public Sequence {
public:
    /** @brief Constructeur. @param robot Instance du robot. */
    Sequence1(Robot& robot) : Sequence(robot) {}
    /** @brief Exécute le parcours en escalier. */
    void execute() override;
};

#endif // SEQUENCE1_H