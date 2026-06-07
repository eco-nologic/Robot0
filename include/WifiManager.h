#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <WebServer.h>
#include "ConfigHardware.h"

class Robot;

// DÉFENSE : "Pourquoi externaliser le WebServer dans une classe WifiManager ?"
// RÉPONSE : "Pour désencombrer le fichier main.cpp et encapsuler toute la logique réseau et la génération HTML/CSV dans un module dédié, facilitant la maintenance."

/**
 * @brief Singleton gérant le serveur Web et la connectivité WiFi.
 */
class WifiManager {
public:
    /** @brief Accès à l'instance unique. */
    static WifiManager& getInstance();
    
    /** @brief Initialise le point d'accès et les routes HTTP. @param robot Référence vers le robot pour les commandes. */
    void begin(Robot& robot);
    /** @brief Gère les requêtes clients (doit être appelé dans loop()). */
    void update();

    WifiManager(const WifiManager&) = delete;
    WifiManager& operator=(const WifiManager&) = delete;

private:
    WifiManager();
    
    Robot* _robot;
    WebServer _server;
    void _setupHandlers();
};

#endif // WIFI_MANAGER_H