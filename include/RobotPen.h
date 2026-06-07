#ifndef ROBOT_PEN_H
#define ROBOT_PEN_H

/**
 * @brief Structure to represent the robot's pen position and orientation in 2D space.
 */
struct RobotPen {
    float x;     // Position X du stylo (cm)
    float y;     // Position Y du stylo (cm)
    float theta; // Orientation du robot (radians)
};

#endif // ROBOT_PEN_H