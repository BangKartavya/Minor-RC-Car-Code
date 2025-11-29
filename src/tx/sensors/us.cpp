#include "us.h"

double UltraSonic::GetDistance(int TRIG, int ECHO) {
    digitalWrite(TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG, LOW);

    int pulseDuration = pulseIn(ECHO, HIGH, 50000);
    double distance = (0.0343 * pulseDuration) / 2;

    return distance == 0 ? 1e9 : distance;
}

void UltraSonic::Init() {
    pinMode(RIGHT_US_TRIG, OUTPUT);
    pinMode(RIGHT_US_ECHO, INPUT);

    pinMode(LEFT_US_TRIG, OUTPUT);
    pinMode(LEFT_US_ECHO, INPUT);

    pinMode(FRONT_US_TRIG, OUTPUT);
    pinMode(FRONT_US_ECHO, INPUT);

    digitalWrite(FRONT_US_TRIG, LOW);
    digitalWrite(LEFT_US_TRIG, LOW);
    digitalWrite(RIGHT_US_TRIG, LOW);
}