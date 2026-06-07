#ifndef PID_CONFIG_H
#define PID_CONFIG_H

/**
 * @brief Structure to hold PID controller parameters and state.
 */
struct PIDController {
    float Kp;
    float Ki;
    float Kd;
    float error_prev;
    float integral;
};

#endif // PID_CONFIG_H