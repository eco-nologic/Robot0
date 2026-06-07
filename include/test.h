#ifndef TEST_H
#define TEST_H

#include <Arduino.h>

// --- Broches Moteurs ---
#define IN_1_D 19
#define IN_2_D 18
#define IN_1_G 17
#define IN_2_G 16

// --- Variables externes (définies dans main.cpp) ---
extern volatile long ticks_G;
extern volatile long ticks_D;
extern int pwm_base;

// --- Constantes du robot ---
extern const float diametre_roue;
extern const int ticksParRotation;
extern const float entraxe;
extern const float rouePerimetre;

// --- Fonctions externes (définies dans main.cpp) ---
void stopMoteurs();
void updateOdometryAfterMovement(float distance_center, float delta_theta);

// --- Fonctions de test ---
void bouger_ticks_en_1s(int ticks_cible_G, int ticks_cible_D, int vitesse_G, int vitesse_D, float delai_G, float delai_D, float override_k = -1.0);

#endif
