#ifndef TEST_H
#define TEST_H

#include <Arduino.h>
#include "ConfigHardware.h"
#include "Robot.h"

// --- Variables externes (définies dans main.cpp) ---
extern Robot myRobot;

// --- Fonctions de test ---
void bouger_ticks_en_1s(int ticks_cible_G, int ticks_cible_D, int vitesse_G, int vitesse_D, float delai_G, float delai_D, float override_k = -1.0);

#endif
