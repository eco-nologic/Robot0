#include "WifiManager.h"
#include <WiFi.h>
#include "Robot.h"
#include "CommandDispatcher.h"
#include "Telemetry.h"
#include "MotorUtility.h"

WifiManager& WifiManager::getInstance() {
    static WifiManager instance;
    return instance;
}

WifiManager::WifiManager() : _robot(nullptr), _server(80) {}

void WifiManager::begin(Robot& robot) {
    _robot = &robot;
    
    WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("WiFi Manager: SoftAP started. IP: ");
    Serial.println(WiFi.softAPIP());

    _setupHandlers();
    _server.begin();
}

void WifiManager::update() {
    _server.handleClient();
}

void WifiManager::_setupHandlers() {
    _server.on("/", [this]() {
        String page = "<html><head><style>body{font-family:Arial;margin:40px;text-align:center;} "
                      "input[type=submit]{font-size:20px;padding:15px;margin:10px;width:320px;cursor:pointer;border-radius:8px;border:none;box-shadow: 0 4px #999;} "
                      "input[type=submit]:active{box-shadow: 0 2px #666; transform: translateY(2px);}"
                      ".stop{background-color:#d9534f;color:white;font-weight:bold;}</style></head><body>"
                      "<h1>🤖 GyroBot Control Center</h1>"
                      "<form action=\"/stop\" method=\"post\"><input type=\"submit\" class=\"stop\" value=\"🛑 ARRÊT D'URGENCE\"></form>"
                      "<hr>"
                      "<h2>📜 Séquences de Dessin</h2>"
                      "<form action=\"/sequence1\" method=\"post\"><input type=\"submit\" value=\"1️⃣ Escalier (Lignes & 90&deg;)\"></form>"
                      "<form action=\"/sequence2\" method=\"post\"><input type=\"submit\" value=\"2️⃣ Cercle (Rotation 390&deg;)\"></form>"
                      "<form action=\"/sequence3\" method=\"post\"><input type=\"submit\" value=\"3️⃣ Boussole (Calibration & Nord)\"></form>"
                      "<form action=\"/fleche\" method=\"post\"><input type=\"submit\" value=\"🎯 Dessiner uniquement la Flèche\"></form>"
                      "<hr><h2>🛠️ Utilitaires Bas Niveau</h2>"
                      "<form action=\"/angle_droit\" method=\"post\"><input type=\"submit\" value=\"📐 Angle Droit (Simulateur)\"></form>"
                      "</body></html>";
        _server.send(200, "text/html", page);
    });

    _server.on("/telemetry", HTTP_GET, [this]() {
        if (_robot) Telemetry::getInstance().update(*_robot);
        // DÉFENSE: "Pourquoi envoyer du CSV au lieu du JSON ?"
        // RÉPONSE: "Pour l'efficacité et la réduction de la charge CPU/RAM."
        _server.send(200, "text/csv", Telemetry::getInstance().getCSV());
    });

    _server.on("/sequence1", HTTP_POST, [this]() {
        _server.send(200, "text/plain", "Lancement Séquence 1 : Escalier");
        CommandDispatcher::getInstance().dispatch("S1");
    });

    _server.on("/sequence2", HTTP_POST, [this]() {
        _server.send(200, "text/plain", "Lancement Séquence 2 : Cercle");
        CommandDispatcher::getInstance().dispatch("S2");
    });

    _server.on("/sequence3", HTTP_POST, [this]() {
        _server.send(200, "text/plain", "Lancement Séquence 3 : Calibration & Nord");
        CommandDispatcher::getInstance().dispatch("S3");
    });

    _server.on("/fleche", HTTP_POST, [this]() {
        _server.send(200, "text/plain", "Tracé de la Flèche en cours...");
        CommandDispatcher::getInstance().dispatch("FLECHE");
    });

    _server.on("/angle_droit", HTTP_POST, [this]() {
        _server.send(200, "text/plain", "Exécution utilitaire Angle Droit");
        CommandDispatcher::getInstance().dispatch("ANGLE_DROIT");
    });

    _server.on("/stop", HTTP_POST, [this]() {
        _server.send(200, "text/plain", "STOPPED");
        CommandDispatcher::getInstance().dispatch("STOP");
    });

    _server.on("/test_ticks", HTTP_POST, [this]() {
        int tg = 0, td = 0, dir_g = 1, dir_d = 1, vg = 80, vd = 80;
        float delai_g = 0.0, delai_d = 0.0;

        if (_server.hasArg("tg")) tg = _server.arg("tg").toInt();
        if (_server.hasArg("td")) td = _server.arg("td").toInt();
        if (_server.hasArg("dir_g")) dir_g = _server.arg("dir_g").toInt();
        if (_server.hasArg("dir_d")) dir_d = _server.arg("dir_d").toInt();
        if (_server.hasArg("vg")) vg = _server.arg("vg").toInt();
        if (_server.hasArg("vd")) vd = _server.arg("vd").toInt();
        if (_server.hasArg("delai_g")) delai_g = _server.arg("delai_g").toFloat();
        if (_server.hasArg("delai_d")) delai_d = _server.arg("delai_d").toFloat();

        int ticks_G_final = tg * dir_g;
        int ticks_D_final = td * dir_d;

        _server.send(200, "text/plain", "Test ticks started");
        bouger_ticks_en_1s(ticks_G_final, ticks_D_final, vg, vd, delai_g, delai_d);
    });

    // PID Tuning endpoints
    _server.on("/pid_fwd", HTTP_POST, [this]() {
        float kp = 2.0f, ki = 0.1f, kd = 0.5f;
        if (_server.hasArg("kp")) kp = _server.arg("kp").toFloat();
        if (_server.hasArg("ki")) ki = _server.arg("ki").toFloat();
        if (_server.hasArg("kd")) kd = _server.arg("kd").toFloat();

        String cmd = "PID_FWD:" + String(kp, 2) + "," + String(ki, 2) + "," + String(kd, 2);
        CommandDispatcher::getInstance().dispatch(cmd);
        _server.send(200, "text/plain", "PID Forward updated: Kp=" + String(kp) + ", Ki=" + String(ki) + ", Kd=" + String(kd));
    });

    _server.on("/pid_rev", HTTP_POST, [this]() {
        float kp = 5.0f, ki = 0.0f, kd = 0.0f;
        if (_server.hasArg("kp")) kp = _server.arg("kp").toFloat();
        if (_server.hasArg("ki")) ki = _server.arg("ki").toFloat();
        if (_server.hasArg("kd")) kd = _server.arg("kd").toFloat();

        String cmd = "PID_REV:" + String(kp, 2) + "," + String(ki, 2) + "," + String(kd, 2);
        CommandDispatcher::getInstance().dispatch(cmd);
        _server.send(200, "text/plain", "PID Reverse updated: Kp=" + String(kp) + ", Ki=" + String(ki) + ", Kd=" + String(kd));
    });

    _server.on("/pwm_base", HTTP_POST, [this]() {
        int pwm = 80;
        if (_server.hasArg("pwm")) pwm = _server.arg("pwm").toInt();

        String cmd = "PWM_BASE:" + String(pwm);
        CommandDispatcher::getInstance().dispatch(cmd);
        _server.send(200, "text/plain", "PWM Base updated: " + String(pwm));
    });

    _server.on("/pid_reset", HTTP_POST, [this]() {
        CommandDispatcher::getInstance().dispatch("PID_RESET");
        _server.send(200, "text/plain", "PID values reset to defaults");
    });
}