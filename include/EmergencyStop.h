#ifndef EMERGENCY_STOP_H
#define EMERGENCY_STOP_H

#include <Arduino.h>

// DÉFENSE : "Comment l'arrêt d'urgence est-il implémenté ?"
// RÉPONSE : "Via un Singleton d'état. Chaque boucle de mouvement longue consulte ce drapeau via isTriggered() pour s'interrompre immédiatement, garantissant une sécurité logicielle réactive."

class EmergencyStop {
public:
    /** @brief Accès à l'instance unique. */
    static EmergencyStop& getInstance();

    /** @brief Active le drapeau d'arrêt d'urgence. */
    void trigger();
    /** @brief Réinitialise le drapeau pour permettre un nouveau mouvement. */
    void reset();
    /** @brief Vérifie si l'arrêt d'urgence a été demandé. */
    bool isTriggered() const;

    EmergencyStop(const EmergencyStop&) = delete;
    EmergencyStop& operator=(const EmergencyStop&) = delete;

private:
    EmergencyStop();
    bool _triggered;
};

#endif // EMERGENCY_STOP_H
