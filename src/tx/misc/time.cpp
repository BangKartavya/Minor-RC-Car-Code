#include "time.h"

double Time::ComputeDt() {
    static unsigned long last = micros();
    unsigned long now = micros();

    unsigned long us = now - last;
    last = now;

    double dt = us * 1e-6;

    if(dt <= 0.0f || dt > 0.05f)
        dt = 0.01f;

    return dt;
}