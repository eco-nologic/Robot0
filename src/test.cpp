#include "test.h"
#include <Arduino.h>

void bouger_ticks_en_1s(int ticks_cible_G, int ticks_cible_D, int vitesse_G, int vitesse_D, float delai_G, float delai_D, float override_k) {
  // On travaille avec les valeurs absolues pour la régulation
  long cible_G = abs(ticks_cible_G);
  long cible_D = abs(ticks_cible_D);

  // Remise à zéro des compteurs de ticks
  ticks_G = 0;
  ticks_D = 0;

  unsigned long delai_G_ms = (unsigned long)(delai_G * 1000.0);
  unsigned long delai_D_ms = (unsigned long)(delai_D * 1000.0);
  
  unsigned long debut_boucle = millis();
  unsigned long fin_G = delai_G_ms + 1000;
  unsigned long fin_D = delai_D_ms + 1000;
  unsigned long duree_totale = max(fin_G, fin_D);
  
  // Paramètres PID dynamiques : doux pour les longues lignes, agressifs pour les petits virages
  float K_temps_G = (cible_G >= 100) ? 1.5 : 4.0;
  if (override_k > 0) K_temps_G = override_k;
  int pwm_min_G   = (cible_G >= 100) ? 0   : 120;
  
  float K_temps_D = (cible_D >= 100) ? 1.5 : 4.0;
  if (override_k > 0) K_temps_D = override_k;
  int pwm_min_D   = (cible_D >= 100) ? 0   : 120;

  while (millis() - debut_boucle < duree_totale) {
    unsigned long temps_ecoule = millis() - debut_boucle;
    
    // ----- Gestion de la roue GAUCHE -----
    if (temps_ecoule < delai_G_ms) {
      analogWrite(IN_1_G, 0); analogWrite(IN_2_G, 0);
      ticks_G = 0; // Ignore les mouvements parasites avant le départ
    } else if (temps_ecoule <= fin_G) {
      unsigned long temps_G_actif = temps_ecoule - delai_G_ms;
      if (temps_G_actif == 0) temps_G_actif = 1;
      
      long consigne_virtuelle_G = (cible_G * temps_G_actif) / 1000;
      long erreur_G = consigne_virtuelle_G - ticks_G;
      int commande_G = 0;
      
      if (cible_G > 0 && erreur_G > 0) {
        commande_G = vitesse_G + (erreur_G * K_temps_G);
        if (commande_G < pwm_min_G) commande_G = pwm_min_G;
        commande_G = constrain(commande_G, 0, 255);
      } else {
        commande_G = 0; // Si en avance, on coupe le moteur
      }
      
      if (ticks_cible_G >= 0) {
        analogWrite(IN_1_G, commande_G); analogWrite(IN_2_G, 0);
      } else {
        analogWrite(IN_1_G, 0); analogWrite(IN_2_G, commande_G);
      }
    } else {
      analogWrite(IN_1_G, 0); analogWrite(IN_2_G, 0);
    }
    
    // ----- Gestion de la roue DROITE -----
    if (temps_ecoule < delai_D_ms) {
      analogWrite(IN_1_D, 0); analogWrite(IN_2_D, 0);
      ticks_D = 0;
    } else if (temps_ecoule <= fin_D) {
      unsigned long temps_D_actif = temps_ecoule - delai_D_ms;
      if (temps_D_actif == 0) temps_D_actif = 1;
      
      long consigne_virtuelle_D = (cible_D * temps_D_actif) / 1000;
      long erreur_D = consigne_virtuelle_D - ticks_D;
      int commande_D = 0;
      
      if (cible_D > 0 && erreur_D > 0) {
        commande_D = vitesse_D + (erreur_D * K_temps_D);
        if (commande_D < pwm_min_D) commande_D = pwm_min_D;
        commande_D = constrain(commande_D, 0, 255);
      } else {
        commande_D = 0;
      }
      
      if (ticks_cible_D >= 0) {
        analogWrite(IN_1_D, 0); analogWrite(IN_2_D, commande_D);
      } else {
        analogWrite(IN_1_D, commande_D); analogWrite(IN_2_D, 0);
      }
    } else {
      analogWrite(IN_1_D, 0); analogWrite(IN_2_D, 0);
    }

    delay(20); // Fréquence de rafraîchissement de la régulation (50 Hz)
  }

  // 2. Arrêt immédiat des moteurs
  stopMoteurs();

  // 3. Mise à jour de l'odométrie
  float dist_G = ((float)ticks_cible_G / ticksParRotation) * rouePerimetre;
  float dist_D = ((float)ticks_cible_D / ticksParRotation) * rouePerimetre;
  float distance_centre = (dist_G + dist_D) / 2.0;
  float delta_theta = (dist_D - dist_G) / entraxe;

  updateOdometryAfterMovement(distance_centre, delta_theta);

  // Affichage de contrôle dans le moniteur série
  Serial.printf("Fin -> Ticks G lus: %ld/%d | Ticks D lus: %ld/%d\n", ticks_G, ticks_cible_G, ticks_D, ticks_cible_D);
}
