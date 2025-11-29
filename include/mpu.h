#ifdef BUILD_TX
#pragma once

#include <Arduino.h>
#include <MPU6050.h>

#define ALPHA 0.98       // MPU Trust Factor
#define GYRO_SCALE 131.0 // from MPU6050 datasheet (LSB per °/s)

#define MPU_SCL 22 // ORANGE
#define MPU_SDA 19 // YELLOW

struct Angle {
        double pitch;
        double yaw;
        double roll;
};

struct Acceleration {
        double x;
        double y;
        double z;
};

class MPU {
    private:
        MPU();

    public:
        static void Init();
        static void GetAccValues();
        static void CalibrateGyro(int samples);
        static void GetAngle(double dt);
        static void ResetYaw();

    public:
        static Angle Old;
        static Angle Offset;
        static MPU6050 Mpu;
        static Acceleration Acc;
};

#endif