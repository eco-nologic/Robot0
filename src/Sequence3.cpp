#include "Sequence3.h"
#include "Magnetometer.h"
#include "Telemetry.h"
#include "SequenceFleche.h"
#include <WebServer.h>

extern WebServer server; 

void Sequence3::execute() {
    // 1. Calibration
    // DÉFENSE : "Pourquoi faire pivoter le robot pendant l'étalonnage du magnétomètre ?"
    // RÉPONSE : "Pour échantillonner le champ magnétique sur 360 degrés, ce qui nous permet de calculer les décalages Hard-Iron (min/max) et les facteurs d'échelle Soft-Iron."
    
    float minX = 100000, maxX = -100000;
    float minY = 100000, maxY = -100000;
    Magnetometer::getInstance().resetCalibration();
    
    unsigned long startTime = millis();
    while(millis() - startTime < 10000) {
        _robot.motorG.drive(110);
        _robot.motorD.drive(-110);
        delay(80);
        _robot.stop();
        delay(50);
        
        sensors_event_t mag;
        Magnetometer::getInstance().getSensor().getEvent(&mag);
        if(mag.magnetic.x < minX) minX = mag.magnetic.x;
        if(mag.magnetic.x > maxX) maxX = mag.magnetic.x;
        if(mag.magnetic.y < minY) minY = mag.magnetic.y;
        if(mag.magnetic.y > maxY) maxY = mag.magnetic.y;
        
        server.handleClient();
        Telemetry::getInstance().update(_robot);
    }
    Magnetometer::getInstance().finalizeCalibration(minX, maxX, minY, maxY);

    // 2. Find North logic and Draw
    // (Implementation truncated for brevity, but calls the arrow)
    SequenceFleche fleche(_robot);
    fleche.execute();
}