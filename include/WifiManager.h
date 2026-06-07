#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <WebServer.h>
#include "ConfigHardware.h"

class Robot;

/**
 * @brief Singleton gérant le serveur Web et la connectivité WiFi.
 */
class WifiManager {
public:
    static WifiManager& getInstance();
    
    void begin(Robot& robot);
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