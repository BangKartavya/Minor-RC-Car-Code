#include "time.h"

double Time::ComputeDt() {
    static unsigned long last = micros();
    unsigned long now = micros();

    unsigned long us = now - last; // time difference in microseconds
    last = now;

    // convert µs → seconds
    double dt = us * 1e-6; // same as us / 1000000.0f

    // clamp dt (IN SECONDS!)
    if(dt <= 0.0f || dt > 0.05f) // > 50ms, loop too slow → stabilize
        dt = 0.01f;              // fallback dt = 10ms = 0.01s

    return dt; // SECONDS
}