#ifndef CONFIG_HARDWARE_H
#define CONFIG_HARDWARE_H

#include <Arduino.h>

// DÉFENSE : "Pourquoi centraliser les définitions des broches dans un fichier d'en-tête ?"
// RÉPONSE : "Cela évite les 'nombres magiques' dans le code source, facilite le portage matériel et garantit que les conflits de broches sont visibles en un coup d'œil."

// --- Motor Pins ---
#define PIN_EN_D 23
#define PIN_EN_G 4
#define PIN_IN1_D 19
#define PIN_IN2_D 18
#define PIN_IN1_G 17
#define PIN_IN2_G 16

// --- Encoder Pins ---
#define PIN_ENC_D_A 27
#define PIN_ENC_G_A 32

// --- I2C Pins ---
#define PIN_I2C_SDA 21
#define PIN_I2C_SCL 22

// --- Robot Physical Parameters ---
#define ROBOT_DIAMETRE_ROUE 9.0f
#define ROBOT_TICKS_PAR_ROTATION 1000
#define ROBOT_ENTRAXE 8.75f
#define ROBOT_ROUE_PERIMETRE (PI * ROBOT_DIAMETRE_ROUE)
#define ROBOT_STYLO_OFFSET 13.0f

// --- WiFi Credentials ---
#define WIFI_SSID "GyroBot_WIFI"
#define WIFI_PASSWORD "12345678"

#endif