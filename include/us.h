#ifdef BUILD_TX
#pragma once
#include <Arduino.h>

#define FRONT_US_TRIG 23 // BLACK
#define FRONT_US_ECHO 21 // GRAY

#define LEFT_US_TRIG 4  // PURPLE
#define LEFT_US_ECHO 34 // GRAY

#define RIGHT_US_TRIG 5  // RED
#define RIGHT_US_ECHO 35 // BROWN

class UltraSonic {
    private:
        UltraSonic();

    public:
        static double GetDistance(int TRIG, int ECHO);
        static void Init();
};

#endif