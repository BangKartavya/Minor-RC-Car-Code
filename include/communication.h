#ifdef BUILD_TX
#pragma once

#include <Arduino.h>
#include <Wifi.h>
#include <esp_now.h>

struct OdomPacket {
        float px;
        float py;

        float vx;
        float vy;

        float yaw;

        float ax;
        float ay;
        float az;

        uint32_t timestamp;
};

class Communication {
    private:
        Communication();

    public:
        static void Init();
        static void SendOdom();

    private:
        static bool ESPNOWReady;
        static uint8_t broadcastAddr[];
};

#endif