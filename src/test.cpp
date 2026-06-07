#include "test.h"
#include <Arduino.h>
#include "Telemetry.h"
#include <WebServer.h>

extern WebServer server;

void bouger_ticks_en_1s(int ticks_cible_G, int ticks_cible_D, int vitesse_G, int vitesse_D, float delai_G, float delai_D, float override_k) {
  // On travaille avec les valeurs absolues pour la régulation
  long cible_G = abs(ticks_cible_G);
  long cible_D = abs(ticks_cible_D);

  // Remise à zéro des compteurs de ticks
  Telemetry::getInstance().setTargetCommand(0, 0); // Manual override ticks
  Telemetry::getInstance().setMoving(true);

  myRobot.motorG.resetTicks();
  myRobot.motorD.resetTicks();

  unsigned long delai_G_ms = (unsigned long)(delai_G * 1000.0);
  unsigned long delai_D_ms = (unsigned long)(delai_D * 1000.0);
  
  unsigned long debut_boucle = millis();
  unsigned long fin_G = delai_G_ms + 1000;
  unsigned long fin_D = delai_D_ms + 1000;
  unsigned long duree_totale = max(fin_G, fin_D);
  
  // DÉFENSE : "Pourquoi utiliser des paramètres PID dynamiques ici ?"
  // RÉPONSE : "Les longues distances nécessitent de la stabilité (K faible), tandis que les virages courts et précis exigent une grande agressivité (K élevé) pour surmonter le frottement statique et le jeu des engrenages."
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
      myRobot.motorG.stop();
      myRobot.motorG.resetTicks();
    } else if (temps_ecoule <= fin_G) {
      unsigned long temps_G_actif = temps_ecoule - delai_G_ms;
      if (temps_G_actif == 0) temps_G_actif = 1;
      
      long consigne_virtuelle_G = (cible_G * temps_G_actif) / 1000;
      long erreur_G = consigne_virtuelle_G - myRobot.motorG.getTicks();
      int commande_G = 0;
      
      if (cible_G > 0 && erreur_G > 0) {
        commande_G = vitesse_G + (erreur_G * K_temps_G);
        if (commande_G < pwm_min_G) commande_G = pwm_min_G;
        commande_G = constrain(commande_G, 0, 255);
      } else {
        commande_G = 0; // Si en avance, on coupe le moteur
      }
      
      if (ticks_cible_G >= 0) {
        myRobot.motorG.drive(commande_G);
      } else {
        myRobot.motorG.drive(-commande_G);
      }
    } else {
      myRobot.motorG.stop();
    }
    
    // ----- Gestion de la roue DROITE -----
    if (temps_ecoule < delai_D_ms) {
      myRobot.motorD.stop();
      myRobot.motorD.resetTicks();
    } else if (temps_ecoule <= fin_D) {
      unsigned long temps_D_actif = temps_ecoule - delai_D_ms;
      if (temps_D_actif == 0) temps_D_actif = 1;
      
      long consigne_virtuelle_D = (cible_D * temps_D_actif) / 1000;
      long erreur_D = consigne_virtuelle_D - myRobot.motorD.getTicks();
      int commande_D = 0;
      
      if (cible_D > 0 && erreur_D > 0) {
        commande_D = vitesse_D + (erreur_D * K_temps_D);
        if (commande_D < pwm_min_D) commande_D = pwm_min_D;
        commande_D = constrain(commande_D, 0, 255);
      } else {
        commande_D = 0;
      }
      
      if (ticks_cible_D >= 0) {
        myRobot.motorD.drive(commande_D);
      } else {
        myRobot.motorD.drive(-commande_D);
      }
    } else {
      myRobot.motorD.stop();
    }

    server.handleClient();
    Telemetry::getInstance().update(myRobot);
    delay(20); // Fréquence de rafraîchissement de la régulation (50 Hz)
  }

  // 2. Arrêt immédiat des moteurs
  myRobot.stop();

  // 3. Mise à jour de l'odométrie
  float dist_G = ((float)ticks_cible_G / ROBOT_TICKS_PAR_ROTATION) * ROBOT_ROUE_PERIMETRE;
  float dist_D = ((float)ticks_cible_D / ROBOT_TICKS_PAR_ROTATION) * ROBOT_ROUE_PERIMETRE;
  float distance_centre = (dist_G + dist_D) / 2.0;
  float delta_theta = (dist_D - dist_G) / ROBOT_ENTRAXE;

  myRobot.updateOdometry(distance_centre, delta_theta);

  // Affichage de contrôle dans le moniteur série
  Serial.printf("Fin -> Ticks G lus: %ld/%d | Ticks D lus: %ld/%d\n", myRobot.motorG.getTicks(), ticks_cible_G, myRobot.motorD.getTicks(), ticks_cible_D);
}
