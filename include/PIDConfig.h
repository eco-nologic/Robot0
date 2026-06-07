#ifndef PID_CONFIG_H
#define PID_CONFIG_H

// DÉFENSE : "Pourquoi utiliser une structure pour le PID ?"
// RÉPONSE : "Pour permettre la gestion de plusieurs contrôleurs indépendants (marche avant, marche arrière) avec un code générique et structuré."

/**
 * @brief Structure to hold PID controller parameters and state.
 */
struct PIDController {
    float Kp;
    float Ki;
    float Kd;
    float error_prev;
    float integral;
};

#endif // PID_CONFIG_H