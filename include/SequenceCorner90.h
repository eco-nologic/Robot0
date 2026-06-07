#ifndef SEQUENCE_CORNER90_H
#define SEQUENCE_CORNER90_H

#include "Sequence.h"

/**
 * @brief Séquence 4 : Virage fluide à 90°.
 * Cette classe encapsule l'appel vers la logique de virage complexe stockée dans le Dispatcher.
 */
class SequenceCorner90 : public Sequence {
public:
    /** @brief Constructeur. @param robot Instance du robot. */
    SequenceCorner90(Robot& robot) : Sequence(robot) {}
    /** @brief Exécute le virage en appelant le code centralisé. */
    void execute() override;
};

#endif // SEQUENCE_CORNER90_H