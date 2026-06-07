#ifndef IKMOVE_H
#define IKMOVE_H

#include <Arduino.h>

/**
 * @brief Représente un segment de mouvement d'1 seconde pour bouger_ticks_en_1s.
 * Correspond à 50 pas de cinématique inverse (Tc=20ms chacun) accumulés.
 * Référence : Shih & Lin, Robotics 2017, eq. 8 et Fig. 2.
 */
struct IKMove {
    int   tg;   // Ticks cibles roue gauche
    int   td;   // Ticks cibles roue droite
    int   vg;   // Vitesse PWM roue gauche (0-255)
    int   vd;   // Vitesse PWM roue droite (0-255)
    float dg;   // Délai départ roue gauche (s)
    float dd;   // Délai départ roue droite (s)

    /** @brief Convertit en commande BLE/WiFi. Ex: "IK_MOVE:45,38,80,80,0.00,0.00" */
    String toCommand() const;

    /** @brief Parse depuis une chaîne "tg,td,vg,vd,dg,dd". */
    static IKMove fromParams(const String& params);
};

#endif // IKMOVE_H
