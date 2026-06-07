//Site
#include <WiFi.h>
#include <WebServer.h>
//Seq3
#include <Wire.h>
#include <SparkFunLSM6DS3.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>
#include "test.h"

// === Définition des broches ===
#define EN_D 23
#define EN_G 4

#define ENC_D_CH_A 27
#define ENC_G_CH_A 32

volatile long ticks_G = 0;
volatile long ticks_D = 0;

// --- Odométrie du stylo (position et orientation) ---
float pose_x = 0;      // Position X du stylo (cm)
float pose_y = 0;      // Position Y du stylo (cm)
float pose_theta = 0;  // Orientation du robot (radians)

// --- Parametres robots ---
extern const float diametre_roue = 9; // cm
extern const int ticksParRotation = 1000;
extern const float entraxe = 8.75; // cm (mesuré centre à centre des roues)
extern const float rouePerimetre = PI * diametre_roue;
const float stylo_offset = 13.0; // Distance du stylo devant le centre des roues arrières (cm)

// --- PID ---
float Kp = 2.0;
float Ki = 0.1;
float Kd = 0.5;
float error_prev = 0;
float integral = 0;

// --- PID R ---
float Kp_R = 5;
float Ki_R = 0;
float Kd_R = 0;
float error_prev_R = 0;
float integral_R = 0;


// --- PWM de base ---
int pwm_base = 80;

// --- Wi-Fi ---
const char* ssid = "GyroBot_WIFI";
const char* password = "12345678";
WebServer server(80);

// --- CAPTEURS ---
LSM6DS3 myIMU(I2C_MODE, 0x6B);  // Gyro/accéléro
Adafruit_LIS3MDL lis3mdl;       // Magnétomètre

// Offsets magnéto (seront recalculés par la calibration)
float offsetX = 0;
float offsetY = 0;
float offsetZ = 0;
float scaleY = 1.0;  // Correction soft-iron (ratio des amplitudes X/Y)
bool calibrated = false;

// Offset de montage du capteur (en degrés)
// Le capteur n'est pas aligné avec l'avant du robot
float headingOffset = -70.0;  // Ajustable depuis la page web

// --- ÉTAT POUR DÉCLENCHEMENT BOUSSOLE ---
bool actionDemandee = true;

// --- Forward declarations ---
void avancer_distance(float cm);
void reculer_distance(float cm);
void tournerAngle(float angleDeg, int pwm_vitesse);
void sequence1();
void sequence2();
void sequence3();
void dessinerFleche();

// --- Interruptions Encodeurs ---
void IRAM_ATTR countTicksDroit() { ticks_D++; }
void IRAM_ATTR countTicksGauche() { ticks_G++; }

// --- Fonctions moteurs ---
void stopMoteurs() {
  analogWrite(IN_1_D, 0); analogWrite(IN_2_D, 0);
  analogWrite(IN_1_G, 0); analogWrite(IN_2_G, 0);
}

// --- Fonction pour normaliser les angles ---
float normalizeAngle(float angle) {
  while (angle > PI) angle -= 2.0 * PI;
  while (angle < -PI) angle += 2.0 * PI;
  return angle;
}

// --- Fonction de mise à jour odométrie avec offset stylo ---
void updateOdometryAfterMovement(float distance_center, float delta_theta) {
  // Mise à jour de l'orientation
  pose_theta = normalizeAngle(pose_theta + delta_theta);
  
  // Mise à jour de la position du centre des roues
  float rear_x = pose_x - stylo_offset * cosf(pose_theta);
  float rear_y = pose_y - stylo_offset * sinf(pose_theta);
  
  rear_x += distance_center * cosf(pose_theta);
  rear_y += distance_center * sinf(pose_theta);
  
  // Convertir la position des roues en position du stylo
  pose_x = rear_x + stylo_offset * cosf(pose_theta);
  pose_y = rear_y + stylo_offset * sinf(pose_theta);
  
  Serial.printf("Pose - X: %.2f, Y: %.2f, Theta: %.2f rad\n", pose_x, pose_y, pose_theta);
}

// --- CALCUL DE CAP ---
float lireCapNord() {
  sensors_event_t mag;
  lis3mdl.getEvent(&mag);

  float mx_cal = mag.magnetic.x - offsetX;
  float my_cal = (mag.magnetic.y - offsetY) * scaleY;  // Correction soft-iron

  float heading = atan2(my_cal, mx_cal) * 180.0 / PI;
  if (heading < 0) heading += 360;
  
  // Appliquer l'offset de montage du capteur
  heading += headingOffset;
  if (heading < 0) heading += 360;
  if (heading >= 360) heading -= 360;
  
  return heading;
}

// --- Interface Web ---
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
  sequence1();
}

void handleSequence2() {
  server.send(200, "text/plain", "Séquence 2 lancée");
  sequence2();
}

void handleSequence3() {
  server.send(200, "text/plain", "Séquence 3 lancée");
  sequence3();
}

void handleFleche() {
  server.send(200, "text/plain", "Dessin de la flèche lancé");
  dessinerFleche();
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

  Wire.begin(21, 22);  // SDA, SCL

  pinMode(EN_D, OUTPUT); pinMode(EN_G, OUTPUT);
  pinMode(IN_1_D, OUTPUT); pinMode(IN_2_D, OUTPUT);
  pinMode(IN_1_G, OUTPUT); pinMode(IN_2_G, OUTPUT);

  pinMode(ENC_D_CH_A, INPUT_PULLUP);
  pinMode(ENC_G_CH_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_D_CH_A), countTicksDroit, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_G_CH_A), countTicksGauche, RISING);

  digitalWrite(EN_D, HIGH);
  digitalWrite(EN_G, HIGH);

  // Capteurs
  if (!lis3mdl.begin_I2C(0x1E)) {
    Serial.println("LIS3MDL non détecté !");
    while (1);
  }

  if (myIMU.begin() != 0) {
    Serial.println("❌ Impossible d'initialiser LSM6DS3 !");
    while (1);
  }

  lis3mdl.setPerformanceMode(LIS3MDL_ULTRAHIGHMODE);
  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  lis3mdl.setDataRate(LIS3MDL_DATARATE_10_HZ);
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);

  WiFi.softAP(ssid, password);
  Serial.print("Wi-Fi actif. IP : ");
  Serial.println(WiFi.softAPIP());

  server.on("/", handleRoot);
  server.on("/sequence1", HTTP_POST, handleSequence1);
  server.on("/sequence2", HTTP_POST, handleSequence2);
  server.on("/sequence3", HTTP_POST, handleSequence3);
  server.on("/fleche", HTTP_POST, handleFleche);
  server.on("/test_ticks", HTTP_POST, handleTestTicks);
  server.on("/angle_droit", HTTP_POST, handleAngleDroit);
  server.begin();
}

void loop() {
  server.handleClient();
}

void tournerAngle(float angleDeg, int pwm_vitesse = 80) {
  float L = entraxe;
  float rayonRotation = L / 2.0;
  float arc = 2 * PI * rayonRotation * (abs(angleDeg) / 360.0);
  long ticks = (arc / rouePerimetre) * ticksParRotation;

  ticks_D = 0;
  ticks_G = 0;

  while ((abs(ticks_D) + abs(ticks_G)) / 2 < ticks) {
    if (angleDeg > 0) {
      analogWrite(IN_1_G, 0); analogWrite(IN_2_G, pwm_vitesse);
      analogWrite(IN_1_D, 0); analogWrite(IN_2_D, pwm_vitesse);
      
    } else {
      analogWrite(IN_1_G, pwm_vitesse); analogWrite(IN_2_G, 0);
      analogWrite(IN_1_D, pwm_vitesse); analogWrite(IN_2_D, 0);
    }

    Serial.print("Ticks G: "); Serial.print(ticks_G);
    Serial.print(" | Ticks D: "); Serial.println(ticks_D);

    delay(10);
  }

  stopMoteurs();
  
  // Mise à jour odométrie: rotation sur place
  float delta_theta = angleDeg * PI / 180.0;  // Convertir en radians
  updateOdometryAfterMovement(0.0, delta_theta);  // distance = 0, rotation uniquement
  
  Serial.println("Rotation sur place terminée.");
  delay(1000);
}

void avancer_distance(float cm) {
  int ticks_target = (int)((cm / rouePerimetre) * ticksParRotation);
  ticks_D = 0;
  ticks_G = 0;
  error_prev = 0;
  integral = 0;

  while (ticks_D < ticks_target || ticks_G < ticks_target) {
    long error = ticks_G - ticks_D;
    integral += error;
    integral = constrain(integral, -500, 500); // anti-windup
    float derivative = error - error_prev;
    float correction = Kp * error + Ki * integral + Kd * derivative;
    error_prev = error;

    int speedG = pwm_base;
    int speedD = pwm_base + correction;

    speedD = constrain(speedD, 0, 255);
    speedG = constrain(speedG, 0, 255);

    analogWrite(IN_1_D, 0); analogWrite(IN_2_D, speedD);
    analogWrite(IN_1_G, speedG); analogWrite(IN_2_G, 0);

    Serial.print("Ticks D: "); Serial.print(ticks_D);
    Serial.print(" | Ticks G: "); Serial.print(ticks_G);
    Serial.print(" | Correction: "); Serial.println(correction);

    delay(10);
  }

  stopMoteurs();
  
  // Mise à jour odométrie: mouvement avant avec correction
  float distance_avg = cm;  // Distance réelle parcourue selon les encodeurs
  updateOdometryAfterMovement(distance_avg, 0.0);  // No rotation during straight movement
  
  Serial.println("Avance terminée.");
  delay(1000);
}

void reculer_distance(float cm) {
  int ticks_target = (int)((cm / rouePerimetre) * ticksParRotation);
  ticks_D = 0;
  ticks_G = 0;
  error_prev_R = 0;
  integral_R = 0;

  while (ticks_D < ticks_target && ticks_G < ticks_target) {
    long error_R = ticks_G - ticks_D;
    integral_R += error_R;
    integral_R = constrain(integral_R, -500, 500); // anti-windup
    float derivative_R = error_R - error_prev_R;
    float correction_R = Kp_R * error_R + Ki_R * integral_R + Kd_R * derivative_R;
    error_prev_R = error_R;

    int speedG = pwm_base;
    int speedD = pwm_base + correction_R;

    speedD = constrain(speedD, 50, 255);
    speedG = constrain(speedG, 50, 255);

    analogWrite(IN_1_D, speedD); analogWrite(IN_2_D, 0); // roue droite recule
    analogWrite(IN_1_G, 0); analogWrite(IN_2_G, speedG); // roue gauche recule

    delay(10);
  }

  stopMoteurs();
  
  // Mise à jour odométrie: mouvement arrière (négatif)
  float distance_avg = -cm;  // Négatif car on recule
  updateOdometryAfterMovement(distance_avg, 0.0);  // No rotation during straight movement
  
  Serial.println("Recul terminé.");
  delay(1000);
}

void tournerDroite() {
  analogWrite(IN_2_D, 90); analogWrite(IN_1_D, 0);
  analogWrite(IN_1_G, 0);  analogWrite(IN_2_G, 90);
}

void sequence1() {
  // === Séquence 1 : Escalier (selon PDF PRJ_SB_v2026_v2) ===
  // Avancer 20 cm, tourner 90° gauche, avancer 10 cm, tourner 90° droite, avancer 40 cm
  // Note : le stylo trace des arcs (R=13cm) aux virages — inévitable avec stylo à l'avant
  
  delay(2000);  // Pause initiale - marquer le départ
  
  // Segment 1 : 20 cm tout droit
  avancer_distance(20);
  delay(500);
  
  // Virage 1 : 90° à gauche (+90 = gauche)
  tournerAngle(90, 80);
  delay(500);
  
  // Segment 2 : 10 cm
  avancer_distance(10);
  delay(500);
  
  // Virage 2 : 90° à droite (-90 = droite)
  tournerAngle(-90, 80);
  delay(500);
  
  // Segment 3 : 40 cm tout droit
  avancer_distance(40);
  
  // Marquer l'arrivée
  stopMoteurs();
  delay(2000);
}

void sequence2() {
  // === Séquence 2 : Cercle ===
  tournerAngle(390, 90);
  delay(2000);
}

void sequence3() {
  // === Séquence 3 : Calibrage auto, pointer vers le Nord, et dessiner une flèche ===
  
  // 1. Calibrage automatique
  Serial.println("Debut du calibrage auto (10s)...");
  float minX = 100000, maxX = -100000;
  float minY = 100000, maxY = -100000;
  
  offsetX = 0; offsetY = 0;
  scaleY = 1.0;
  
  unsigned long startTime = millis();
  while(millis() - startTime < 10000) {
    // Rotation pour calibrer (PWM à 110 pour être sûr de bien tourner sur 360°)
    analogWrite(IN_1_G, 110); analogWrite(IN_2_G, 0);
    analogWrite(IN_1_D, 110); analogWrite(IN_2_D, 0);
    delay(80);
    stopMoteurs();
    delay(50);
    
    sensors_event_t mag;
    lis3mdl.getEvent(&mag);
    if(mag.magnetic.x < minX) minX = mag.magnetic.x;
    if(mag.magnetic.x > maxX) maxX = mag.magnetic.x;
    if(mag.magnetic.y < minY) minY = mag.magnetic.y;
    if(mag.magnetic.y > maxY) maxY = mag.magnetic.y;
    
    server.handleClient();
  }
  
  offsetX = (maxX + minX) / 2.0;
  offsetY = (maxY + minY) / 2.0;
  float rangeX = maxX - minX;
  float rangeY = maxY - minY;
  scaleY = (rangeY > 0.1) ? (rangeX / rangeY) : 1.0;
  calibrated = true;
  
  Serial.println("Calibrage termine.");

  // 2. Recherche du Nord
  Serial.println("Recherche du nord...");
  
  while (true) {
    float heading = lireCapNord();
    float error = heading;
    if (error > 180) error -= 360;
    
    Serial.printf("Cap: %.2f | Erreur: %.2f\n", heading, error);
    
    // Tolérance de ±5 degrés
    if (abs(error) <= 5.0) {
      Serial.println("Nord detecte ! Arret immediat.");
      break;
    }
    
    // Calcul de l'angle et des ticks (utilisation de bouger_ticks avec l'anti-stall)
    float rayonRotation = entraxe / 2.0;
    float arc = 2 * PI * rayonRotation * (abs(error) / 360.0);
    int ticks = (arc / rouePerimetre) * ticksParRotation;
    
    // Si l'erreur est très petite, on force au moins un mini-mouvement pour vaincre l'incertitude
    if (ticks < 10) ticks = 10;
    
    if (error > 0) {
      // Pour tourner à GAUCHE: reculer roue gauche, avancer roue droite
      bouger_ticks_en_1s(-ticks, ticks, 80, 80, 0.0, 0.0, 4.0);
    } else {
      // Pour tourner à DROITE: avancer roue gauche, reculer roue droite
      bouger_ticks_en_1s(ticks, -ticks, 80, 80, 0.0, 0.0, 4.0);
    }
    
    delay(500); // Pause pour laisser le robot se stabiliser et le magnétomètre se calmer
    server.handleClient();
  }
  
  stopMoteurs();
  delay(1000);

  // 3. Dessiner la fleche
  Serial.println("Nord trouve.");
  dessinerFleche();

  Serial.println("Sequence 3 terminee.");
}

void dessinerFleche() {
  Serial.println("Dessin de la fleche.");
  avancer_distance(10);
  delay(500);

  bouger_ticks_en_1s(-16, 20, 100, 100, 0.0, 0.0, 4.0);
  bouger_ticks_en_1s(50, 20, 120, 120, 0.0, 0.0, 4.0);
  bouger_ticks_en_1s(50, 20, 120, 120, 0.0, 0.0, 4.0);
  bouger_ticks_en_1s(-20, -40, 120, 120, 0.0, 0.0, 4.0);
  bouger_ticks_en_1s(-20, -40, 120, 120, 0.0, 0.0, 4.0);
  bouger_ticks_en_1s(-20, 20, 100, 100, 0.0, 0.0, 4.0);
  bouger_ticks_en_1s(-10, 20, 100, 100, 0.0, 0.0, 4.0);
  bouger_ticks_en_1s(40, -20, 100, 100, 0.0, 0.0, 4.0);
  bouger_ticks_en_1s(-20, 30, 100, 100, 0.0, 0.0, 4.0);
  bouger_ticks_en_1s(28, -15, 100, 100, 0.0, 0.0, 4.0);
  bouger_ticks_en_1s(-16, 22, 100, 100, 0.0, 0.0, 4.0);
  bouger_ticks_en_1s(22, -10, 100, 100, 0.0, 0.0, 4.0);
  bouger_ticks_en_1s(-12, 15, 100, 100, 0.0, 0.0, 4.0);
  bouger_ticks_en_1s(15, -8, 100, 100, 0.0, 0.0, 4.0);
  bouger_ticks_en_1s(-8, 12, 100, 100, 0.0, 0.0, 4.0);
  bouger_ticks_en_1s(12, -6, 100, 100, 0.0, 0.0, 4.0);
  bouger_ticks_en_1s(-5, 10, 100, 100, 0.0, 0.0, 4.0);
  bouger_ticks_en_1s(8, -4, 100, 100, 0.0, 0.0, 4.0);
  bouger_ticks_en_1s(-3, 6, 100, 100, 0.0, 0.0, 4.0);
}
