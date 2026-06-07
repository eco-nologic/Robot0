#ifndef SEQUENCE_FLECHE_H
#define SEQUENCE_FLECHE_H

#include "Sequence.h"

/**
 * @brief Séquence utilitaire pour dessiner une flèche.
 * Utilise des commandes de bas niveau (ticks) pour tracer la géométrie d'une flèche complexe.
 */
class SequenceFleche : public Sequence {
public:
    /** @brief Constructeur. @param robot Instance du robot. */
    SequenceFleche(Robot& robot) : Sequence(robot) {}
    /** @brief Exécute le tracé de la flèche. */
    void execute() override;
};

#endif // SEQUENCE_FLECHE_H