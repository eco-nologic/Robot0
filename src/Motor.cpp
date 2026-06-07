#include "Motor.h"

Motor::Motor(int pinEn, int pinIn1, int pinIn2, int pinEnc, bool inverted)
    : _pinEn(pinEn), _pinIn1(pinIn1), _pinIn2(pinIn2), _pinEnc(pinEnc), 
      _inverted(inverted), _ticks(0) {}

void Motor::begin() {
    pinMode(_pinEn, OUTPUT);
    pinMode(_pinIn1, OUTPUT);
    pinMode(_pinIn2, OUTPUT);
    pinMode(_pinEnc, INPUT_PULLUP);
    
    digitalWrite(_pinEn, HIGH); // Enable the driver
    stop();
}

// DÉFENSE : "Pourquoi gérer l'inversion de direction (bool inverted) directement dans la classe Motor ?"
// RÉPONSE : "Les moteurs étant montés en miroir sur le châssis, une commande 'avant' ferait tourner le robot sur lui-même. En gérant l'inversion ici, le code de haut niveau (PID/Odométrie) utilise des valeurs positives intuitives pour avancer."

void Motor::drive(int speed) {
    speed = constrain(speed, -255, 255);
    
    // If speed is negative, we reverse. 
    // The _inverted flag handles physical mounting differences between Left/Right.
    bool reverse = (speed < 0);
    if (_inverted) reverse = !reverse;

    int absSpeed = abs(speed);

    if (reverse) {
        analogWrite(_pinIn1, absSpeed);
        analogWrite(_pinIn2, 0);
    } else {
        analogWrite(_pinIn1, 0);
        analogWrite(_pinIn2, absSpeed);
    }
}

void Motor::stop() {
    analogWrite(_pinIn1, 0);
    analogWrite(_pinIn2, 0);
}