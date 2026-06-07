#ifndef SEQUENCE2_H
#define SEQUENCE2_H

#include "Sequence.h"

/**
 * @brief Séquence 2 : Tracé d'un cercle.
 * Effectue une rotation de 390° pour valider la précision circulaire et l'odométrie.
 */
class Sequence2 : public Sequence {
public:
    /** @brief Constructeur. @param robot Instance du robot. */
    Sequence2(Robot& robot) : Sequence(robot) {}
    /** @brief Exécute la rotation circulaire. */
    void execute() override;
};

#endif // SEQUENCE2_H