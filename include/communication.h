#pragma once
#include <Arduino.h>

// Call once in setup()
void commInit();

// Call every loop() (very lightweight)
void commSendOdom();

// Structure of the odometry packet
struct OdomPacket {
        float px;
        float py;
        float vx;
        float vy;
        float yaw;
        uint32_t timestamp;
};
