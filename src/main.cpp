//Site
//Seq3
#include <Wire.h>
#include <SparkFunLSM6DS3.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>
#include "MotorUtility.h"
#include "PIDConfig.h"
#include "RobotPen.h"
#include "Magnetometer.h"
#include "GyroAccel.h"
#include "Motor.h"
#include "Robot.h"
#include "Sequences.h"
#include "Telemetry.h"
#include "BleManager.h"
#include "EmergencyStop.h"
#include "CommandDispatcher.h"
#include "WifiManager.h"

#define ENABLE_WIFI false
#define ENABLE_BLE true

Robot myRobot;

// --- ÉTAT POUR DÉCLENCHEMENT BOUSSOLE ---
// DÉFENSE : "Pourquoi utiliser IRAM_ATTR pour les interruptions des encodeurs ?"
// RÉPONSE : "Pour placer le code en RAM plutôt qu'en Flash, réduisant ainsi la latence et garantissant que nous ne manquons pas d'impulsions d'encodeur à haute vitesse."
bool actionDemandee = true;

// --- Interruptions Encodeurs ---
void IRAM_ATTR countTicksDroit() { myRobot.motorD.handleInterrupt(); }
void IRAM_ATTR countTicksGauche() { myRobot.motorG.handleInterrupt(); }

// --- CALCUL DE CAP ---
float lireCapNord() {
  return Magnetometer::getInstance().getHeading();
}

void processBleCommand(const String& command) {
  CommandDispatcher::getInstance().dispatch(command);
}

void setup() {
  Serial.begin(115200);

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

  myRobot.begin();

  CommandDispatcher::getInstance().begin(myRobot);

  attachInterrupt(digitalPinToInterrupt(PIN_ENC_D_A), countTicksDroit, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_G_A), countTicksGauche, RISING);

  if (!Magnetometer::getInstance().begin()) {
    Serial.println("LIS3MDL non détecté !");
    while (1);
  }
  if (!GyroAccel::getInstance().begin()) {
    Serial.println("❌ Impossible d'initialiser LSM6DS3 !");
    while (1);
  }

  if (ENABLE_WIFI) {
    WifiManager::getInstance().begin(myRobot);
  }

  if (ENABLE_BLE) {
    BleManager::getInstance().begin(myRobot);
    Serial.println("BLE activated");
  }
}

void loop() {
  Telemetry::getInstance().update(myRobot);

  if (ENABLE_WIFI) {
    WifiManager::getInstance().update();
  }

  if (ENABLE_BLE) {
    BleManager::getInstance().update();

    if (BleManager::getInstance().hasCommand()) {
      String cmd = BleManager::getInstance().getNextCommand();
      processBleCommand(cmd);
    }

    if (BleManager::getInstance().isClientConnected()) {
      String csvData = Telemetry::getInstance().getCSV();
      BleManager::getInstance().notifyTelemetry(csvData);
    }
  }
}
