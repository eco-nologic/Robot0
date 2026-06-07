#ifndef SEQUENCE_H
#define SEQUENCE_H

#include "Robot.h"

// DÉFENSE : "Pourquoi utiliser une classe de base abstraite avec une méthode virtuelle execute() ?"
// RÉPONSE : "C'est l'application du principe Ouvert/Fermé (SOLID). On peut ajouter de nouvelles formes de dessin (nouvelles classes) sans jamais modifier le code du serveur Web ou de la classe Robot."

/**
 * @brief Abstract base class for all robot movement sequences.
 */
class Sequence {
protected:
    Robot& _robot; ///< Reference to the robot instance to be controlled.

public:
    /**
     * @brief Construct a new Sequence object.
     * @param robot The robot instance.
     */
    Sequence(Robot& robot) : _robot(robot) {}
    virtual ~Sequence() {}

    /** 
     * @brief Méthode virtuelle pure à implémenter par chaque séquence. 
     * Contient la logique de mouvement spécifique (ex: carré, cercle).
     */
    virtual void execute() = 0;
};

#endif // SEQUENCE_H