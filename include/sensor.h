#ifdef BUILD_TX
#pragma once

#include "gps.h"
#include "us.h"
#include "mpu.h"

#define CLEAR_MARGIN 35 // what we consider "clear"
#define ZUPT_WIN 20     // window size for ZUPT detection (samples)
#define ZUPT_VAR_THRESH 0.02f
#define ZUPT_MAG_THRESH 0.25f
#define TURN_CHECK_DELAY 10 // ms per integration step

class Sensor {
    private:
        Sensor();

    public:
        static void Init();
        static bool DetectZUPTWindow();
        static void IMUZUPT();
        static void LatLonToXY(double lat, double lon, float& x, float& y);
        static void Update(double dt);

    public:
        static float ZUPTBuff[ZUPT_WIN][3];
        static int ZuptIdx;
        static float PosX;
        static float PosY;
        static float PosXIMU;
        static float PosYIMU;
        static float VelX;
        static float VelY;
};

#endif