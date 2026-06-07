#ifndef SEQUENCE3_H
#define SEQUENCE3_H

#include "Sequence.h"

/**
 * @brief Séquence 3 : Calibration Magnétomètre et Flèche Nord.
 * Réalise une calibration hard-iron/soft-iron puis pointe vers le Nord pour dessiner une flèche.
 */
class Sequence3 : public Sequence {
public:
    /** @brief Constructeur. @param robot Instance du robot. */
    Sequence3(Robot& robot) : Sequence(robot) {}
    /** @brief Exécute la calibration et le pointage Nord. */
    void execute() override;
};

#endif // SEQUENCE3_H