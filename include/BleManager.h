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

class BleManager : public BLEServerCallbacks, public BLECharacteristicCallbacks {
public:
    static BleManager& getInstance();

    void begin(Robot& robot);
    void stop();
    void update();
    void notifyTelemetry(const String& csvData);
    bool isClientConnected() const;
    bool hasCommand() const;
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
