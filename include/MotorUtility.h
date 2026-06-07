#ifndef MOTOR_UTILITY_H
#define MOTOR_UTILITY_H

#include <Arduino.h>
#include "ConfigHardware.h"
#include "Robot.h"

// --- Variables externes (définies dans main.cpp) ---
extern Robot myRobot;

// DÉFENSE : "Quel est le rôle de bouger_ticks_en_1s ?"
// RÉPONSE : "C'est une fonction de bas niveau permettant des mouvements asymétriques précis sur une durée fixe, indispensable pour tracer des courbes complexes impossibles en PID ligne droite standard."

// --- Fonctions utilitaires moteurs ---
/**
 * @brief Déplace le robot en spécifiant un nombre de ticks cible pour chaque roue sur une durée théorique d'une seconde.
 */
void bouger_ticks_en_1s(int ticks_cible_G, int ticks_cible_D, int vitesse_G, int vitesse_D, float delai_G, float delai_D, float override_k = -1.0);

#endif