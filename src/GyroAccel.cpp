#include "GyroAccel.h"
#include <EEPROM.h>

#define HEADING_OFFSET_ADDR 0  // EEPROM address for heading offset (4 bytes)

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
    bool success = imu.begin() == IMU_SUCCESS;
    // Load heading offset from EEPROM on startup
    if (success) {
        loadHeadingOffsetFromEEPROM();
    }
    return success;
}

void GyroAccel::saveHeadingOffsetToEEPROM() {
    EEPROM.begin(512);
    union {
        float fval;
        uint8_t bytes[4];
    } data;
    data.fval = headingOffset;
    for (int i = 0; i < 4; i++) {
        EEPROM.write(HEADING_OFFSET_ADDR + i, data.bytes[i]);
    }
    EEPROM.commit();
    EEPROM.end();
}

void GyroAccel::loadHeadingOffsetFromEEPROM() {
    EEPROM.begin(512);
    union {
        float fval;
        uint8_t bytes[4];
    } data;
    for (int i = 0; i < 4; i++) {
        data.bytes[i] = EEPROM.read(HEADING_OFFSET_ADDR + i);
    }
    EEPROM.end();
    // Only load if value looks valid (not all 0xFF from empty EEPROM)
    if (data.bytes[0] != 0xFF || data.bytes[1] != 0xFF) {
        headingOffset = data.fval;
    }
}