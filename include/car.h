#ifdef BUILD_TX
#pragma once

#include <Arduino.h>
#include "sensor.h"
#include "time.h"

#define IN1 14 // BLACK
#define IN2 27 // WHITE
#define IN3 26 // GRAY
#define IN4 25 // PURPLE
#define ENA 12 // BLUE
#define ENB 13 // GREEN

class Car {
    private:
        Car();

    public:
        static void Init();
        static void Left();
        static void Right();
        static void Forward();
        static void Backward();
        static void Stop();
        static void HandleCommand(char cmd);
        static void TurnAngle(float targetAngle, bool leftTurn);

    public:
        static int Speed;
        static int ShouldBeAutomated;
};

#endif