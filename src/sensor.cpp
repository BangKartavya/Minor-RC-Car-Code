#include "sensor.h"

MPU6050 mpu(0x68);
float accArr[3] = {0, 0, 0};

double getDistance(int TRIG, int ECHO) {
    digitalWrite(TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG, LOW);

    int pulseDuration = pulseIn(ECHO, HIGH, 50000);
    double distance = (0.0343 * pulseDuration) / 2;

    return distance;
}

void getAccValues() {
    int16_t ax = 0, ay = 0, az = 0;
    int16_t gx = 0, gy = 0, gz = 0;

    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    float ax_g = ax / 16384.0;
    float ay_g = ay / 16384.0;
    float az_g = az / 16384.0;

    float gx_dps = gx / 131.0;
    float gy_dps = gy / 131.0;
    float gz_dps = gz / 131.0;

    accArr[0] = ax_g;
    accArr[1] = ay_g;
    accArr[2] = az_g;
}
