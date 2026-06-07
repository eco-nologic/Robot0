#include "GyroAccel.h"

GyroAccel& GyroAccel::getInstance() {
    static GyroAccel instance;
    return instance;
}

GyroAccel::GyroAccel() : imu(I2C_MODE, 0x6B) {
    // Constructor uses the provided initialization parameters
}

bool GyroAccel::begin() {
    /**
     * begin() returns IMU_SUCCESS (0) on success in the SparkFun library.
     * We convert this to a boolean for consistency.
     */
    return imu.begin() == IMU_SUCCESS;
}