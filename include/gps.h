#ifdef BUILD_TX
#pragma once

#include <TinyGPS++.h>

#define GPS_RX 33 // connect to NEO-6M TX YELLOW
#define GPS_TX 32 // connect to NEO-6M RX BLUE

class GPS {
    private:
        GPS();

    public:
        static void Update();
        static void Init();

    public:
        static HardwareSerial GPS_Serial;
        static String NMEALine;
        static TinyGPSPlus Gps;
        static bool OriginSet;
        static double OriginLat;
        static double OriginLon;
};

#endif