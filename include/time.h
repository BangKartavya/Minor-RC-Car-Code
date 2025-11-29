#ifdef BUILD_TX
#pragma once
#include <Arduino.h>

class Time {
    private:
        Time();

    public:
        static double ComputeDt();
};

#endif