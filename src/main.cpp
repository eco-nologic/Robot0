//Site
#include <WiFi.h>
#include <WebServer.h>
//Seq3
#include <Wire.h>
#include <SparkFunLSM6DS3.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>
#include "test.h"
#include "PIDConfig.h"
#include "RobotPen.h"
#include "Magnetometer.h"
#include "GyroAccel.h"
#include "Motor.h"
#include "Robot.h"
#include "Sequences.h"
#include "Telemetry.h"

Robot myRobot;

WebServer server(80);

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

// --- Interface Web ---
void handleTelemetry() {
  Telemetry::getInstance().update(myRobot);
  server.send(200, "application/json", Telemetry::getInstance().getJSON());
}

void handleRoot() {
  String page = "<html><head><style>body{font-family:Arial;margin:40px;text-align:center;} "
                "input[type=submit]{font-size:20px;padding:15px;margin:10px;}</style></head><body>"
                "<h1>GyroBot - Commandes</h1>"
                "<form action=\"/sequence1\" method=\"post\">"
                "<input type=\"submit\" value=\"Lancer Sequence 1 (Escalier)\">"
                "</form>"
                "<form action=\"/sequence2\" method=\"post\">"
                "<input type=\"submit\" value=\"Lancer Sequence 2 (Cercle)\">"
                "</form>"
                "<form action=\"/sequence3\" method=\"post\">"
                "<input type=\"submit\" value=\"Lancer Sequence 3 (Fleche Nord)\">"
                "</form>"
                "<form action=\"/fleche\" method=\"post\">"
                "<input type=\"submit\" value=\"Dessiner uniquement la Fleche\">"
                "</form>"
                "<hr>"
                "<h2>Séquences Personnalisées</h2>"
                "<form action=\"/angle_droit\" method=\"post\">"
                "<input type=\"submit\" value=\"Angle Droit\">"
                "</form>"
                "</body></html>";
  server.send(200, "text/html", page);
}

void handleSequence1() {
  server.send(200, "text/plain", "Séquence 1 lancée");
  Sequence1 s(myRobot);
  s.execute();
}

void handleSequence2() {
  server.send(200, "text/plain", "Séquence 2 lancée");
  Sequence2 s(myRobot);
  s.execute();
}

void handleSequence3() {
  server.send(200, "text/plain", "Séquence 3 lancée");
  Sequence3 s(myRobot);
  s.execute();
}

void handleFleche() {
  server.send(200, "text/plain", "Dessin de la flèche lancé");
  SequenceFleche s(myRobot);
  s.execute();
}

void handleTestTicks() {
  int tg = 0;
  int td = 0;
  int dir_g = 1;
  int dir_d = 1;
  int vg = 80;
  int vd = 80;
  float delai_g = 0.0;
  float delai_d = 0.0;
  
  if (server.hasArg("tg")) tg = server.arg("tg").toInt();
  if (server.hasArg("td")) td = server.arg("td").toInt();
  if (server.hasArg("dir_g")) dir_g = server.arg("dir_g").toInt();
  if (server.hasArg("dir_d")) dir_d = server.arg("dir_d").toInt();
  if (server.hasArg("vg")) vg = server.arg("vg").toInt();
  if (server.hasArg("vd")) vd = server.arg("vd").toInt();
  if (server.hasArg("delai_g")) delai_g = server.arg("delai_g").toFloat();
  if (server.hasArg("delai_d")) delai_d = server.arg("delai_d").toFloat();
  
  // Appliquer les directions
  int ticks_G_final = tg * dir_g;
  int ticks_D_final = td * dir_d;
  
  String msg = "Test moteurs lance avec :<br>";
  msg += "Gauche -> Ticks: " + String(ticks_G_final) + ", Vitesse: " + String(vg) + ", Delai: " + String(delai_g) + "s<br>";
  msg += "Droite -> Ticks: " + String(ticks_D_final) + ", Vitesse: " + String(vd) + ", Delai: " + String(delai_d) + "s<br>";
  msg += "<a href=\"/\">Retour</a>";
  
  server.send(200, "text/html", msg);
  
  bouger_ticks_en_1s(ticks_G_final, ticks_D_final, vg, vd, delai_g, delai_d);
}

void handleAngleDroit() {
  server.send(200, "text/html", "Séquence Angle Droit lancée !<br><a href=\"/\">Retour</a>");
  
  // Séquences générées par le simulateur
  bouger_ticks_en_1s(353, 353, 80, 80, 0.0, 0.0);
  bouger_ticks_en_1s(30, -10, 100, 100, 0.0, 0.0);
  bouger_ticks_en_1s(50, -10, 100, 100, 0.0, 0.0);
  bouger_ticks_en_1s(50, 1, 100, 70, 0.0, 0.5);
  bouger_ticks_en_1s(50, 1, 100, 70, 0.0, 0.4);
  bouger_ticks_en_1s(30, 50, 100, 100, 0.0, 0.0, 4.00);
  bouger_ticks_en_1s(33, 50, 100, 100, 0.0, 0.0, 3.94);
  bouger_ticks_en_1s(35, 50, 100, 100, 0.0, 0.0, 3.89);
  bouger_ticks_en_1s(37, 50, 100, 100, 0.0, 0.0, 3.83);
  bouger_ticks_en_1s(37, 50, 100, 100, 0.0, 0.0, 3.78);
  bouger_ticks_en_1s(39, 50, 100, 100, 0.0, 0.0, 3.72);
  bouger_ticks_en_1s(39, 50, 100, 100, 0.0, 0.0, 3.67);
  bouger_ticks_en_1s(41, 50, 100, 100, 0.0, 0.0, 3.61);
  bouger_ticks_en_1s(41, 50, 100, 100, 0.0, 0.0, 3.56);
  bouger_ticks_en_1s(43, 50, 100, 100, 0.0, 0.0, 3.50);
  bouger_ticks_en_1s(43, 50, 100, 100, 0.0, 0.0, 3.44);
  bouger_ticks_en_1s(44, 50, 100, 100, 0.0, 0.0, 3.39);
  bouger_ticks_en_1s(45, 50, 100, 100, 0.0, 0.0, 3.33);
  bouger_ticks_en_1s(45, 50, 100, 100, 0.0, 0.0, 3.28);
  bouger_ticks_en_1s(45, 50, 100, 100, 0.0, 0.0, 3.22);
  bouger_ticks_en_1s(46, 50, 100, 100, 0.0, 0.0, 3.17);
  bouger_ticks_en_1s(46, 50, 100, 100, 0.0, 0.0, 3.11);
  bouger_ticks_en_1s(46, 50, 100, 100, 0.0, 0.0, 3.06);
  bouger_ticks_en_1s(47, 50, 100, 100, 0.0, 0.0, 3.00);
}

void setup() {
  Serial.begin(115200);

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

  myRobot.begin();

  attachInterrupt(digitalPinToInterrupt(PIN_ENC_D_A), countTicksDroit, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_G_A), countTicksGauche, RISING);

  // Capteurs
  if (!Magnetometer::getInstance().begin()) {
    Serial.println("LIS3MDL non détecté !");
    while (1);
  }
  if (!GyroAccel::getInstance().begin()) {
    Serial.println("❌ Impossible d'initialiser LSM6DS3 !");
    while (1);
  }

  WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Wi-Fi actif. IP : ");
  Serial.println(WiFi.softAPIP());

  server.on("/", handleRoot);
  server.on("/sequence1", HTTP_POST, handleSequence1);
  server.on("/sequence2", HTTP_POST, handleSequence2);
  server.on("/sequence3", HTTP_POST, handleSequence3);
  server.on("/fleche", HTTP_POST, handleFleche);
  server.on("/test_ticks", HTTP_POST, handleTestTicks);
  server.on("/angle_droit", HTTP_POST, handleAngleDroit);
  server.on("/telemetry", HTTP_GET, handleTelemetry);
  server.begin();
}

void loop() {
  Telemetry::getInstance().update(myRobot);
  server.handleClient();
}
