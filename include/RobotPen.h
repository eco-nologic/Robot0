#ifndef ROBOT_PEN_H
#define ROBOT_PEN_H

// DÉFENSE : "Pourquoi stocker la pose du stylo plutôt que celle du robot ?"
// RÉPONSE : "C'est l'outil terminal. L'odométrie calculée sur le stylo garantit que le dessin correspond aux coordonnées théoriques, malgré le déport physique (offset) par rapport à l'essieu."

/**
 * @brief Structure to represent the robot's pen position and orientation in 2D space.
 */
struct RobotPen {
    float x;     // Position X du stylo (cm)
    float y;     // Position Y du stylo (cm)
    float theta; // Orientation du robot (radians)
};

#endif // ROBOT_PEN_H