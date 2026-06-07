#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

/**
 * @brief Classe représentant un moteur CC avec encodeur.
 * Gère la commande PWM, le sens de rotation et le comptage des ticks.
 */
class Motor {
public:
    /**
     * @brief Constructeur du moteur.
     * @param pinEn Broche Enable (PWM).
     * @param pinIn1 Broche Direction 1.
     * @param pinIn2 Broche Direction 2.
     * @param pinEnc Broche Phase A de l'encodeur.
     * @param inverted Inversion logicielle du sens (pour montage en miroir).
     */
    Motor(int pinEn, int pinIn1, int pinIn2, int pinEnc, bool inverted = false);
    
    /** @brief Configure les broches en entrée/sortie. */
    void begin();
    /** @brief Définit la vitesse et le sens. @param speed Valeur [-255, 255]. */
    void drive(int speed);
    /** @brief Arrête le moteur (freinage/roue libre selon driver). */
    void stop();
    
    /** @brief Récupère le nombre de ticks accumulés. */
    long getTicks() const { return _ticks; }
    /** @brief Remet le compteur de ticks à zéro. */
    void resetTicks() { _ticks = 0; }
    
    // Method called by the ISR wrapper
    void IRAM_ATTR handleInterrupt() { _ticks++; }

    // DÉFENSE : "Pourquoi utiliser 'volatile' pour le compteur de ticks ?"
    // RÉPONSE : "Pour informer le compilateur que _ticks peut changer en dehors du flux normal du programme (à l'intérieur d'une ISR), évitant ainsi des optimisations incorrectes."
private:
    int _pinEn, _pinIn1, _pinIn2, _pinEnc;
    bool _inverted;
    volatile long _ticks;
};

#endif // MOTOR_H