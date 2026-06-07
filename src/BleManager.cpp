#include "BleManager.h"
#include "Robot.h"
#include "EmergencyStop.h"
#include "Sequences.h"

#define SERVICE_UUID           "12345678-1234-1234-1234-123456789abc"
#define CHARACTERISTIC_TX_UUID "87654321-4321-4321-4321-cba987654321"
#define CHARACTERISTIC_RX_UUID "11111111-2222-3333-4444-555555555555"

BleManager& BleManager::getInstance() {
    static BleManager instance;
    return instance;
}

BleManager::BleManager()
    : _robot(nullptr),
      _pServer(nullptr),
      _pTxCharacteristic(nullptr),
      _pRxCharacteristic(nullptr),
      _pAdvertising(nullptr),
      _deviceConnected(false),
      _isAdvertising(false),
      _lastDisconnectTime(0) {}

BleManager::~BleManager() {
    if (_pServer != nullptr) {
        BLEDevice::deinit();
    }
}

void BleManager::begin(Robot& robot) {
    _robot = &robot;

    BLEDevice::init("GyroBot_BLE");
    BLEDevice::setMTU(256);

    _pServer = BLEDevice::createServer();
    _pServer->setCallbacks(this);

    BLEService* pService = _pServer->createService(SERVICE_UUID);

    _pTxCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_TX_UUID,
        BLECharacteristic::PROPERTY_NOTIFY
    );
    _pTxCharacteristic->addDescriptor(new BLE2902());

    _pRxCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_RX_UUID,
        BLECharacteristic::PROPERTY_WRITE
    );
    _pRxCharacteristic->setCallbacks(this);

    pService->start();

    _pAdvertising = BLEDevice::getAdvertising();
    _pAdvertising->addServiceUUID(SERVICE_UUID);
    _pAdvertising->setScanResponse(true);
    _pAdvertising->setMinPreferred(0x06);
    _pAdvertising->setMaxPreferred(0x12);
    _pAdvertising->start();
    _isAdvertising = true;

    Serial.println("BLE Manager initialized and advertising started");
}

void BleManager::stop() {
    if (_pServer != nullptr) {
        BLEDevice::deinit();
        _pServer = nullptr;
    }
}

void BleManager::update() {
    restartAdvertising();
}

void BleManager::notifyTelemetry(const String& csvData) {
    if (_deviceConnected && _pTxCharacteristic != nullptr) {
        _pTxCharacteristic->setValue(csvData.c_str());
        _pTxCharacteristic->notify();
    }
}

bool BleManager::isClientConnected() const {
    return _deviceConnected;
}

void BleManager::onConnect(BLEServer* pServer) {
    _deviceConnected = true;
    Serial.println("BLE Client connected");
}

void BleManager::onDisconnect(BLEServer* pServer) {
    _deviceConnected = false;
    _isAdvertising = false;
    _lastDisconnectTime = millis();
    Serial.println("BLE Client disconnected, re-advertising in 500ms");
}

void BleManager::restartAdvertising() {
    if (!_deviceConnected && !_isAdvertising && _pAdvertising != nullptr) {
        uint32_t timeSinceDisconnect = millis() - _lastDisconnectTime;
        if (timeSinceDisconnect >= ADVERTISING_DELAY_MS) {
            _pAdvertising->start();
            _isAdvertising = true;
            Serial.println("BLE advertising restarted");
        }
    }
}

bool BleManager::hasCommand() const {
    return !_commandQueue.empty();
}

String BleManager::getNextCommand() {
    if (_commandQueue.empty()) return "";
    String cmd = _commandQueue.front();
    _commandQueue.pop();
    return cmd;
}

void BleManager::onWrite(BLECharacteristic* pCharacteristic) {
    std::string value = pCharacteristic->getValue();
    if (value.length() == 0) return;

    String command(value.c_str());
    command.trim();

    if (command == "STOP") {
        Serial.println("BLE Emergency STOP triggered immediately");
        EmergencyStop::getInstance().trigger();
        return;
    }

    Serial.print("BLE Command queued: ");
    Serial.println(command);

    if (_commandQueue.size() < MAX_QUEUE_SIZE) {
        _commandQueue.push(command);
    } else {
        Serial.println("BLE command queue full, command dropped");
    }
}
