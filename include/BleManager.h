#ifndef BLE_MANAGER_H
#define BLE_MANAGER_H

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <queue>
#include "Telemetry.h"

class Robot;

// DÉFENSE : "Pourquoi avoir choisi le format CSV pour la télémétrie BLE ?"
// RÉPONSE : "Pour l'efficacité. Le CSV élimine la verbosité du JSON, permettant de faire tenir une trame complète dans un seul paquet MTU de 256 octets, réduisant ainsi la latence et la consommation CPU."

class BleManager : public BLEServerCallbacks, public BLECharacteristicCallbacks {
public:
    /** @brief Accès à l'instance unique. */
    static BleManager& getInstance();

    /** @brief Initialise le serveur BLE et l'advertising. */
    void begin(Robot& robot);
    /** @brief Arrête proprement le service BLE. */
    void stop();
    /** @brief Gère l'état de l'advertising (reconnexion). */
    void update();
    /** @brief Envoie une ligne de télémétrie aux clients connectés. */
    void notifyTelemetry(const String& csvData);
    /** @brief Indique si un client est connecté. */
    bool isClientConnected() const;
    /** @brief Vérifie la présence de nouvelles commandes dans la file. */
    bool hasCommand() const;
    /** @brief Récupère la prochaine commande à traiter. */
    String getNextCommand();

    void onConnect(BLEServer* pServer) override;
    void onDisconnect(BLEServer* pServer) override;
    void onWrite(BLECharacteristic* pCharacteristic) override;

    BleManager(const BleManager&) = delete;
    BleManager& operator=(const BleManager&) = delete;

private:
    BleManager();
    ~BleManager();

    Robot* _robot;
    BLEServer* _pServer;
    BLECharacteristic* _pTxCharacteristic;
    BLECharacteristic* _pRxCharacteristic;
    BLEAdvertising* _pAdvertising;
    bool _deviceConnected;
    bool _isAdvertising;
    uint32_t _lastDisconnectTime;
    std::queue<String> _commandQueue;

    static const uint32_t ADVERTISING_DELAY_MS = 500;
    static const size_t MAX_QUEUE_SIZE = 10;

    void restartAdvertising();
};

#endif // BLE_MANAGER_H
